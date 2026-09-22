#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"
#include "esp_twai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// default configure (DM4310). 上位パラメータとモーター側の設定が一致している必要がある．
// コンパイルオプション(-DDM_P_MAX=...)などで上書き可能
#ifndef DM_P_MIN
#define DM_P_MIN   (-12.5f)
#endif
#ifndef DM_P_MAX
#define DM_P_MAX   ( 12.5f)
#endif

#ifndef DM_V_MIN
#define DM_V_MIN   (-45.0f)
#endif
#ifndef DM_V_MAX
#define DM_V_MAX   ( 45.0f)
#endif

#ifndef DM_KP_MIN
#define DM_KP_MIN  (0.0f)
#endif
#ifndef DM_KP_MAX
#define DM_KP_MAX  (500.0f)
#endif

#ifndef DM_KD_MIN
#define DM_KD_MIN  (0.0f)
#endif
#ifndef DM_KD_MAX
#define DM_KD_MAX  (5.0f)
#endif

#ifndef DM_T_MIN
#define DM_T_MIN   (-18.0f)
#endif
#ifndef DM_T_MAX
#define DM_T_MAX   ( 18.0f)
#endif

typedef enum
{
    DM_STATE_DISABLE         = 0x0,
    DM_STATE_ENABLE          = 0x1,

    DM_STATE_OVERVOLTAGE     = 0x8, // 過電圧
    DM_STATE_UNDERVOLTAGE    = 0x9, // 低電圧

    DM_STATE_OVERCURRENT     = 0xA, // 過電流

    DM_STATE_MOS_OVER_TEMP   = 0xB, // MOS過熱
    DM_STATE_MOTOR_OVER_TEMP = 0xC, // モータ過熱

    DM_STATE_CAN_TIMEOUT     = 0xD, // CAN通信タイムアウト
    DM_STATE_OVERLOAD        = 0xE, // 過負荷

} dm_state_t;

typedef struct
{
    uint32_t id;
    /**
     *feedback: id|err<<4, pos[15:8], pos[7:0], vel[11:4], vel[3:0]|T11:8], T[7:0], T_MOS, T_Rotor
     */
    float pos; //position
    float vel; //velocity
    float torque;

    dm_state_t state;
    uint8_t motor_temp;
    uint8_t mos_temp;

} dm_feedback_t;

/**
 * @brief TWAI(CAN)ノードを作成して有効化する (1Mbps, 全ID受信)
 * @note  送受信関数はすべてタスクから呼ぶこと (ISRからは不可)
 */
esp_err_t dm_twai_init(gpio_num_t tx, gpio_num_t rx);
esp_err_t dm_twai_deinit(void);
/**
 * @brief バスオフからの復帰を開始する
 */
esp_err_t dm_twai_recover(void);
/**
 * @brief 内部で使用しているTWAIノードのハンドル (未初期化ならNULL)
 */
twai_node_handle_t dm_twai_get_node(void);

esp_err_t dm_transmit(uint16_t can_id, const uint8_t *data, TickType_t ticks_to_wait);
esp_err_t dm_transmit_mit(uint16_t can_id, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait);
esp_err_t dm_transmit_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait);

esp_err_t dm_enable(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_disable(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_pos_init(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_clear_error(uint16_t can_id, TickType_t ticks_to_wait);

esp_err_t dm_receive(dm_feedback_t *fb, TickType_t ticks_to_wait);

void dm_pack_mit_cmd(uint8_t *data, float pos, float vel, float kp, float kd, float torque);

uint32_t dm_float_to_uint(float x, float x_min, float x_max, int bits);
float dm_uint_to_float(uint32_t x, float x_min, float x_max, int bits);

const char *dm_state_to_string(dm_state_t state);

void dm_dump_feedback(const dm_feedback_t *fb);
void dm_dump_twai_status(void);

#ifdef __cplusplus
}
#endif
