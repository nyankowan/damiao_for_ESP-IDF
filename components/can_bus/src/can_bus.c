#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "can_bus.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define CAN_BUS_TAG "can_bus"

// 受信タスクが停止要求を確認する間隔
#define CAN_BUS_RX_POLL_TICKS pdMS_TO_TICKS(100)

typedef struct
{
    uint32_t id;
    uint32_t mask;
    bool extd;
    can_bus_rx_cb_t cb;
    void *ctx;
} can_bus_handler_t;

/*
 * 新TWAIドライバ(esp_driver_twai)は送信フレームをコピーせずポインタのままキューに積むため，
 * 送信完了(on_tx_done)まで twai_frame_t とデータバッファを保持しておく必要がある．
 * 送信はFIFO順に完了するので，リングバッファ + カウンティングセマフォで空きスロットを管理する．
 */
typedef struct
{
    twai_frame_t frame;
    uint8_t data[CAN_BUS_MAX_DATA_LEN];
} can_bus_tx_slot_t;

static twai_node_handle_t s_node = NULL;
static QueueHandle_t s_rx_queue = NULL;
static SemaphoreHandle_t s_tx_free = NULL;  // 空きスロット数
static SemaphoreHandle_t s_tx_lock = NULL;  // s_tx_head の更新と送信順序の保護
static can_bus_tx_slot_t s_tx_slots[CONFIG_CAN_BUS_TX_QUEUE_LEN];
static size_t s_tx_head = 0;

static SemaphoreHandle_t s_handler_lock = NULL; // ハンドラ表とデフォルトハンドラの保護
static can_bus_handler_t s_handlers[CONFIG_CAN_BUS_MAX_HANDLERS];
static size_t s_num_handlers = 0;
static can_bus_rx_cb_t s_default_cb = NULL;
static void *s_default_ctx = NULL;

static TaskHandle_t s_rx_task = NULL;
static volatile bool s_rx_task_stop = false;
static SemaphoreHandle_t s_rx_task_done = NULL;

static volatile uint32_t s_rx_dropped = 0;

static IRAM_ATTR bool can_bus_on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t buf[CAN_BUS_MAX_DATA_LEN] = {0};
    twai_frame_t rx = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) {
        return false;
    }

    can_frame_t frame = {
        .id = rx.header.id,
        .extd = rx.header.ide,
        .rtr = rx.header.rtr,
        // クラシックCANではDLC 9-15も8バイトを意味する
        .len = rx.header.dlc > CAN_BUS_MAX_DATA_LEN ? CAN_BUS_MAX_DATA_LEN : rx.header.dlc,
    };
    if (!frame.rtr) {
        memcpy(frame.data, buf, frame.len);
    }

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &frame, &woken) != pdTRUE) {
        s_rx_dropped++;
    }
    return woken == pdTRUE;
}

static IRAM_ATTR bool can_bus_on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_free, &woken);
    return woken == pdTRUE;
}

static bool can_bus_handler_match(const can_bus_handler_t *h, const can_frame_t *frame)
{
    return h->extd == frame->extd && (frame->id & h->mask) == (h->id & h->mask);
}

static void can_bus_dispatch(const can_frame_t *frame)
{
    // コールバックはロックの外で呼ぶ (コールバックが長引いても登録をブロックしないため)．
    // 登録は追加のみで解除はないので，コピーしたハンドラは呼び出し時点でも有効．
    can_bus_handler_t matched[CONFIG_CAN_BUS_MAX_HANDLERS];
    size_t num_matched = 0;
    can_bus_rx_cb_t default_cb;
    void *default_ctx;

    xSemaphoreTake(s_handler_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_num_handlers; i++) {
        if (can_bus_handler_match(&s_handlers[i], frame)) {
            matched[num_matched++] = s_handlers[i];
        }
    }
    default_cb = s_default_cb;
    default_ctx = s_default_ctx;
    xSemaphoreGive(s_handler_lock);

    for (size_t i = 0; i < num_matched; i++) {
        matched[i].cb(frame, matched[i].ctx);
    }
    if (num_matched == 0 && default_cb) {
        default_cb(frame, default_ctx);
    }
}

static void can_bus_rx_task(void *arg)
{
    can_frame_t frame;
    while (!s_rx_task_stop) {
        if (xQueueReceive(s_rx_queue, &frame, CAN_BUS_RX_POLL_TICKS) == pdTRUE) {
            can_bus_dispatch(&frame);
        }
    }
    xSemaphoreGive(s_rx_task_done);
    vTaskDelete(NULL);
}

esp_err_t can_bus_init(const can_bus_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, CAN_BUS_TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(s_node == NULL, ESP_ERR_INVALID_STATE, CAN_BUS_TAG, "already initialized");

    esp_err_t ret = ESP_OK;
    s_rx_queue = xQueueCreate(CONFIG_CAN_BUS_RX_QUEUE_LEN, sizeof(can_frame_t));
    s_tx_free = xSemaphoreCreateCounting(CONFIG_CAN_BUS_TX_QUEUE_LEN, CONFIG_CAN_BUS_TX_QUEUE_LEN);
    s_tx_lock = xSemaphoreCreateMutex();
    s_handler_lock = xSemaphoreCreateMutex();
    s_rx_task_done = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(s_rx_queue && s_tx_free && s_tx_lock && s_handler_lock && s_rx_task_done,
                      ESP_ERR_NO_MEM, err, CAN_BUS_TAG, "no mem");
    s_tx_head = 0;
    s_num_handlers = 0;
    s_default_cb = NULL;
    s_default_ctx = NULL;
    s_rx_dropped = 0;

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = config->tx,
            .rx = config->rx,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = config->bitrate,
        },
        .fail_retry_cnt = -1, // ACKが返るまで再送
        .tx_queue_depth = CONFIG_CAN_BUS_TX_QUEUE_LEN,
    };
    ESP_GOTO_ON_ERROR(twai_new_node_onchip(&node_config, &s_node), err, CAN_BUS_TAG, "twai_new_node_onchip failed");

    twai_event_callbacks_t cbs = {
        .on_rx_done = can_bus_on_rx_done,
        .on_tx_done = can_bus_on_tx_done,
    };
    ESP_GOTO_ON_ERROR(twai_node_register_event_callbacks(s_node, &cbs, NULL), err, CAN_BUS_TAG, "register callbacks failed");

    s_rx_task_stop = false;
    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(can_bus_rx_task, "can_bus_rx", CONFIG_CAN_BUS_RX_TASK_STACK_SIZE, NULL,
                                              CONFIG_CAN_BUS_RX_TASK_PRIORITY, &s_rx_task, tskNO_AFFINITY) == pdPASS,
                      ESP_ERR_NO_MEM, err, CAN_BUS_TAG, "create rx task failed");

    ESP_GOTO_ON_ERROR(twai_node_enable(s_node), err, CAN_BUS_TAG, "twai_node_enable failed");

    return ESP_OK;

err:
    can_bus_deinit();
    return ret;
}

esp_err_t can_bus_deinit(void)
{
    if (s_node) {
        twai_node_disable(s_node);
    }
    if (s_rx_task) {
        s_rx_task_stop = true;
        xSemaphoreTake(s_rx_task_done, portMAX_DELAY);
        s_rx_task = NULL;
    }
    if (s_node) {
        twai_node_delete(s_node);
        s_node = NULL;
    }
    if (s_rx_queue) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    if (s_tx_free) {
        vSemaphoreDelete(s_tx_free);
        s_tx_free = NULL;
    }
    if (s_tx_lock) {
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
    }
    if (s_handler_lock) {
        vSemaphoreDelete(s_handler_lock);
        s_handler_lock = NULL;
    }
    if (s_rx_task_done) {
        vSemaphoreDelete(s_rx_task_done);
        s_rx_task_done = NULL;
    }
    s_num_handlers = 0;
    s_default_cb = NULL;
    s_default_ctx = NULL;
    return ESP_OK;
}

esp_err_t can_bus_register(uint32_t id, uint32_t mask, bool extd, can_bus_rx_cb_t cb, void *ctx)
{
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, CAN_BUS_TAG, "cb is NULL");
    ESP_RETURN_ON_FALSE(s_handler_lock, ESP_ERR_INVALID_STATE, CAN_BUS_TAG, "not initialized");

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(s_handler_lock, portMAX_DELAY);
    if (s_num_handlers < CONFIG_CAN_BUS_MAX_HANDLERS) {
        s_handlers[s_num_handlers++] = (can_bus_handler_t) {
            .id = id,
            .mask = mask,
            .extd = extd,
            .cb = cb,
            .ctx = ctx,
        };
    } else {
        ret = ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_handler_lock);

    if (ret != ESP_OK) {
        ESP_LOGE(CAN_BUS_TAG, "handler table full (CONFIG_CAN_BUS_MAX_HANDLERS=%d)", CONFIG_CAN_BUS_MAX_HANDLERS);
    }
    return ret;
}

esp_err_t can_bus_set_default_handler(can_bus_rx_cb_t cb, void *ctx)
{
    ESP_RETURN_ON_FALSE(s_handler_lock, ESP_ERR_INVALID_STATE, CAN_BUS_TAG, "not initialized");

    xSemaphoreTake(s_handler_lock, portMAX_DELAY);
    s_default_cb = cb;
    s_default_ctx = ctx;
    xSemaphoreGive(s_handler_lock);
    return ESP_OK;
}

esp_err_t can_bus_transmit(const can_frame_t *frame, TickType_t ticks_to_wait)
{
    ESP_RETURN_ON_FALSE(frame, ESP_ERR_INVALID_ARG, CAN_BUS_TAG, "frame is NULL");
    ESP_RETURN_ON_FALSE(frame->len <= CAN_BUS_MAX_DATA_LEN, ESP_ERR_INVALID_SIZE, CAN_BUS_TAG, "len > 8");
    ESP_RETURN_ON_FALSE(s_node, ESP_ERR_INVALID_STATE, CAN_BUS_TAG, "not initialized");

    if (xSemaphoreTake(s_tx_free, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);

    can_bus_tx_slot_t *slot = &s_tx_slots[s_tx_head];
    memcpy(slot->data, frame->data, frame->len);
    slot->frame = (twai_frame_t) {
        .header = {
            .id = frame->id,
            .dlc = frame->len,
            .ide = frame->extd,
            .rtr = frame->rtr,
        },
        .buffer = slot->data,
        .buffer_len = frame->rtr ? 0 : frame->len,
    };

    // 空きスロットを確保済みなのでドライバの送信キューは溢れない
    esp_err_t e = twai_node_transmit(s_node, &slot->frame, 0);
    if (e == ESP_OK) {
        s_tx_head = (s_tx_head + 1) % CONFIG_CAN_BUS_TX_QUEUE_LEN;
    } else {
        xSemaphoreGive(s_tx_free);
    }

    xSemaphoreGive(s_tx_lock);
    return e;
}

esp_err_t can_bus_recover(void)
{
    ESP_RETURN_ON_FALSE(s_node, ESP_ERR_INVALID_STATE, CAN_BUS_TAG, "not initialized");
    return twai_node_recover(s_node);
}

uint32_t can_bus_get_rx_dropped(void)
{
    return s_rx_dropped;
}

void can_bus_dump_status(void)
{
    if (s_node == NULL) {
        printf("can_bus not initialized\n");
        return;
    }

    twai_node_status_t status;
    twai_node_record_t record;
    if (twai_node_get_info(s_node, &status, &record) != ESP_OK) {
        printf("twai_node_get_info failed\n");
        return;
    }

    printf(
        "state=%d tx_err=%u rx_err=%u bus_error=%" PRIu32 " rx_pending=%u rx_dropped=%" PRIu32 "\n",
        status.state,
        status.tx_error_count,
        status.rx_error_count,
        record.bus_err_num,
        (unsigned)uxQueueMessagesWaiting(s_rx_queue),
        s_rx_dropped
    );
}
