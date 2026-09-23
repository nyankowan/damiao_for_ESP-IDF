#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BUS_MAX_DATA_LEN 8

#define CAN_BUS_STD_ID_MASK 0x7FFu
#define CAN_BUS_EXT_ID_MASK 0x1FFFFFFFu

/**
 * @brief CANフレーム (クラシックCAN)
 *        TWAIドライバの型に依存しないよう，送受信はすべてこの型で行う
 */
typedef struct
{
    uint32_t id;
    bool extd;     // true: 29bit拡張ID, false: 11bit標準ID
    bool rtr;      // リモートフレーム
    uint8_t len;   // データ長 (0-8)
    uint8_t data[CAN_BUS_MAX_DATA_LEN];
} can_frame_t;

/**
 * @brief 受信コールバック
 * @note  can_busの受信タスクから呼ばれる．ブロックする処理や重い処理はしないこと
 *        (その間ほかのフレームの配送が止まる)．
 *        コールバック内から can_bus_register() などの登録系関数を呼んではいけない．
 */
typedef void (*can_bus_rx_cb_t)(const can_frame_t *frame, void *ctx);

typedef struct
{
    gpio_num_t tx;
    gpio_num_t rx;
    uint32_t bitrate; // [bps]
} can_bus_config_t;

#define CAN_BUS_DEFAULT_CONFIG(tx_io, rx_io) { \
    .tx = (tx_io),                             \
    .rx = (rx_io),                             \
    .bitrate = 1000000,                        \
}

/**
 * @brief TWAI(CAN)ノードを作成・有効化し，受信タスクを起動する
 */
esp_err_t can_bus_init(const can_bus_config_t *config);
esp_err_t can_bus_deinit(void);

/**
 * @brief 受信ハンドラを登録する
 *
 * (frame.id & mask) == (id & mask) かつ frame.extd == extd のフレームが cb に渡される．
 * 複数のハンドラに一致した場合はすべてに配送される．
 * 登録の解除はできない (ctx は can_bus_deinit() まで有効なメモリを渡すこと)．
 *
 * @param id   受信したいID
 * @param mask 比較するビット (1つのIDだけなら CAN_BUS_STD_ID_MASK / CAN_BUS_EXT_ID_MASK)
 * @param extd 拡張IDのフレームを対象にするか
 */
esp_err_t can_bus_register(uint32_t id, uint32_t mask, bool extd, can_bus_rx_cb_t cb, void *ctx);

/**
 * @brief どのハンドラにも一致しなかったフレームの受け取り先を設定する (NULLで解除)
 */
esp_err_t can_bus_set_default_handler(can_bus_rx_cb_t cb, void *ctx);

/**
 * @brief フレームを送信する (複数タスクから呼んでよい．ISRからは不可)
 * @param ticks_to_wait 送信キューに空きが出るまで待つ時間
 */
esp_err_t can_bus_transmit(const can_frame_t *frame, TickType_t ticks_to_wait);

/**
 * @brief バスオフからの復帰を開始する
 */
esp_err_t can_bus_recover(void);

/**
 * @brief 受信キューが満杯で破棄したフレーム数
 */
uint32_t can_bus_get_rx_dropped(void);

void can_bus_dump_status(void);

#ifdef __cplusplus
}
#endif
