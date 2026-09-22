#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "damiao.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DAMIAO_TAG "damiao"

#define DM_FRAME_LEN 8

#define DM_CMD_ENABLE      {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}
#define DM_CMD_DISABLE     {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}
#define DM_CMD_POS_INIT    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}
#define DM_CMD_CLEAR_ERROR {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[DM_FRAME_LEN];
} dm_rx_msg_t;

/*
 * 新TWAIドライバ(esp_driver_twai)は送信フレームをコピーせずポインタのままキューに積むため，
 * 送信完了(on_tx_done)まで twai_frame_t とデータバッファを保持しておく必要がある．
 * 送信はFIFO順に完了するので，リングバッファ + カウンティングセマフォで空きスロットを管理する．
 */
typedef struct
{
    twai_frame_t frame;
    uint8_t data[DM_FRAME_LEN];
} dm_tx_slot_t;

static twai_node_handle_t s_node = NULL;
static QueueHandle_t s_rx_queue = NULL;
static SemaphoreHandle_t s_tx_free = NULL;  // 空きスロット数
static SemaphoreHandle_t s_tx_lock = NULL;  // s_tx_head の更新と送信順序の保護
static dm_tx_slot_t s_tx_slots[CONFIG_DM_TWAI_TX_QUEUE_LEN];
static size_t s_tx_head = 0;

static IRAM_ATTR bool dm_on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t buf[DM_FRAME_LEN] = {0};
    twai_frame_t rx = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) {
        return false;
    }

    dm_rx_msg_t msg = {
        .id = rx.header.id,
        .len = rx.header.dlc > DM_FRAME_LEN ? DM_FRAME_LEN : rx.header.dlc,
    };
    memcpy(msg.data, buf, DM_FRAME_LEN);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &msg, &woken); // 満杯なら破棄
    return woken == pdTRUE;
}

static IRAM_ATTR bool dm_on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_free, &woken);
    return woken == pdTRUE;
}

esp_err_t dm_twai_init(gpio_num_t tx, gpio_num_t rx)
{
    ESP_RETURN_ON_FALSE(s_node == NULL, ESP_ERR_INVALID_STATE, DAMIAO_TAG, "already initialized");

    esp_err_t ret = ESP_OK;
    s_rx_queue = xQueueCreate(CONFIG_DM_TWAI_RX_QUEUE_LEN, sizeof(dm_rx_msg_t));
    s_tx_free = xSemaphoreCreateCounting(CONFIG_DM_TWAI_TX_QUEUE_LEN, CONFIG_DM_TWAI_TX_QUEUE_LEN);
    s_tx_lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(s_rx_queue && s_tx_free && s_tx_lock, ESP_ERR_NO_MEM, err, DAMIAO_TAG, "no mem");
    s_tx_head = 0;

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = tx,
            .rx = rx,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CONFIG_DM_TWAI_BITRATE,
        },
        .fail_retry_cnt = -1, // 旧ドライバ(TWAI_MODE_NORMAL)と同じくACKが返るまで再送
        .tx_queue_depth = CONFIG_DM_TWAI_TX_QUEUE_LEN,
    };
    ESP_GOTO_ON_ERROR(twai_new_node_onchip(&node_config, &s_node), err, DAMIAO_TAG, "twai_new_node_onchip failed");

    twai_event_callbacks_t cbs = {
        .on_rx_done = dm_on_rx_done,
        .on_tx_done = dm_on_tx_done,
    };
    ESP_GOTO_ON_ERROR(twai_node_register_event_callbacks(s_node, &cbs, NULL), err, DAMIAO_TAG, "register callbacks failed");
    ESP_GOTO_ON_ERROR(twai_node_enable(s_node), err, DAMIAO_TAG, "twai_node_enable failed");

    return ESP_OK;

err:
    dm_twai_deinit();
    return ret;
}

esp_err_t dm_twai_deinit(void)
{
    if (s_node) {
        twai_node_disable(s_node);
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
    return ESP_OK;
}

esp_err_t dm_twai_recover(void)
{
    ESP_RETURN_ON_FALSE(s_node, ESP_ERR_INVALID_STATE, DAMIAO_TAG, "not initialized");
    return twai_node_recover(s_node);
}

twai_node_handle_t dm_twai_get_node(void)
{
    return s_node;
}

esp_err_t dm_transmit(uint16_t can_id, const uint8_t *data, TickType_t ticks_to_wait)
{
    ESP_RETURN_ON_FALSE(s_node, ESP_ERR_INVALID_STATE, DAMIAO_TAG, "not initialized");

    if (xSemaphoreTake(s_tx_free, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);

    dm_tx_slot_t *slot = &s_tx_slots[s_tx_head];
    memcpy(slot->data, data, DM_FRAME_LEN);
    slot->frame = (twai_frame_t) {
        .header = {
            .id = can_id,
            .dlc = DM_FRAME_LEN,
            .ide = 0,
        },
        .buffer = slot->data,
        .buffer_len = DM_FRAME_LEN,
    };

    // 空きスロットを確保済みなのでドライバの送信キューは溢れない
    esp_err_t e = twai_node_transmit(s_node, &slot->frame, 0);
    if (e == ESP_OK) {
        s_tx_head = (s_tx_head + 1) % CONFIG_DM_TWAI_TX_QUEUE_LEN;
    } else {
        xSemaphoreGive(s_tx_free);
    }

    xSemaphoreGive(s_tx_lock);
    return e;
}

esp_err_t dm_transmit_mit(uint16_t can_id, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait)
{
    uint8_t data[DM_FRAME_LEN];
    dm_pack_mit_cmd(data, pos, vel, kp, kd, torque);

    return dm_transmit(can_id, data, ticks_to_wait);
}

esp_err_t dm_enable(uint16_t can_id, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_ENABLE;
    return dm_transmit(can_id, cmd, ticks_to_wait);
}

esp_err_t dm_pos_init(uint16_t can_id, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_POS_INIT;
    return dm_transmit(can_id, cmd, ticks_to_wait);
}

esp_err_t dm_clear_error(uint16_t can_id, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_CLEAR_ERROR;
    return dm_transmit(can_id, cmd, ticks_to_wait);
}

esp_err_t dm_disable(uint16_t can_id, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_DISABLE;
    return dm_transmit(can_id, cmd, ticks_to_wait);
}

esp_err_t dm_transmit_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait)
{
    uint8_t data[DM_FRAME_LEN];
    dm_pack_mit_cmd(data,
                    0.0f,   // pos
                    0.0f,   // vel
                    0.0f,   // kp
                    0.0f,   // kd
                    torque);

    return dm_transmit(can_id, data, ticks_to_wait);
}

esp_err_t dm_receive(dm_feedback_t *fb, TickType_t ticks_to_wait)
{
    ESP_RETURN_ON_FALSE(s_rx_queue, ESP_ERR_INVALID_STATE, DAMIAO_TAG, "not initialized");

    dm_rx_msg_t rx;
    if (xQueueReceive(s_rx_queue, &rx, ticks_to_wait) != pdTRUE) {
        ESP_LOGV(DAMIAO_TAG, "receive timeout");
        return ESP_ERR_TIMEOUT;
    }
    if (rx.len < DM_FRAME_LEN) {
        ESP_LOGV(DAMIAO_TAG, "invalid length frame (id=0x%" PRIx32 ", len=%d)", rx.id, rx.len);
        return ESP_ERR_INVALID_SIZE;
    }

    fb->id = rx.id;
    uint16_t p_int = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    uint16_t v_int = ((uint16_t)rx.data[3] << 4) | (rx.data[4] >> 4);
    uint16_t t_int = (((uint16_t)rx.data[4] & 0x0F) << 8) | rx.data[5];

    fb->state = rx.data[0] >> 4;
    fb->pos = dm_uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
    fb->vel = dm_uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
    fb->torque = dm_uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    fb->mos_temp   = rx.data[6];
    fb->motor_temp = rx.data[7];

    return ESP_OK;
}

void dm_pack_mit_cmd(uint8_t *data, float pos, float vel, float kp, float kd, float torque)
{
    uint16_t p_int  = dm_float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v_int  = dm_float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_int = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t t_int  = dm_float_to_uint(torque, DM_T_MIN, DM_T_MAX, 12);

    data[0] = p_int >> 8;
    data[1] = p_int;

    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);

    data[4] = kp_int;

    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);

    data[7] = t_int;
}

uint32_t dm_float_to_uint(float x, float x_min, float x_max, int bits)
{
    // 範囲外の値はビット幅を超えて隣のフィールドを壊すので飽和させる
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    float span = x_max - x_min;
    float offset = x_min;

    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float dm_uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x) * span / ((float)((1 << bits) - 1)) + offset;
}

const char *dm_state_to_string(dm_state_t state)
{
    switch(state)
    {
        case DM_STATE_DISABLE:
            return "DISABLE";

        case DM_STATE_ENABLE:
            return "ENABLE";

        case DM_STATE_OVERVOLTAGE:
            return "OVERVOLTAGE";

        case DM_STATE_UNDERVOLTAGE:
            return "UNDERVOLTAGE";

        case DM_STATE_OVERCURRENT:
            return "OVERCURRENT";

        case DM_STATE_MOS_OVER_TEMP:
            return "MOS_OVER_TEMP";

        case DM_STATE_MOTOR_OVER_TEMP:
            return "MOTOR_OVER_TEMP";

        case DM_STATE_CAN_TIMEOUT:
            return "CAN_TIMEOUT";

        case DM_STATE_OVERLOAD:
            return "OVERLOAD";

        default:
            return "UNKNOWN";
    }
}

void dm_dump_feedback(const dm_feedback_t *fb)
{
    printf(
            "rx id=0x%02" PRIX32 " "
            "pos=%.3f "
            "vel=%.3f "
            "torque=%.3f "
            "state=0x%2X "
            "mos_temp=%3d "
            "motor_temp=%3d \n",
            fb->id,
            fb->pos,
            fb->vel,
            fb->torque,
            fb->state,
            fb->mos_temp,
            fb->motor_temp);
}

void dm_dump_twai_status(void)
{
    if (s_node == NULL) {
        printf("twai not initialized\n");
        return;
    }

    twai_node_status_t status;
    twai_node_record_t record;
    if (twai_node_get_info(s_node, &status, &record) != ESP_OK) {
        printf("twai_node_get_info failed\n");
        return;
    }

    printf(
        "state=%d tx_err=%u rx_err=%u bus_error=%" PRIu32 " rx_pending=%u\n",
        status.state,
        status.tx_error_count,
        status.rx_error_count,
        record.bus_err_num,
        (unsigned)uxQueueMessagesWaiting(s_rx_queue)
    );
}
