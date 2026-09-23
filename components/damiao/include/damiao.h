#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "can_bus.h"

#ifdef __cplusplus
extern "C" {
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

/**
 * @brief 位置・速度・トルクの範囲 (MIN = -MAX)
 *        モーター側の設定 (PMAX, VMAX, TMAX) と一致している必要がある
 */
typedef struct
{
    float p_max; // [rad]
    float v_max; // [rad/s]
    float t_max; // [Nm]
} dm_limits_t;

#define DM_LIMITS_DEFAULT { \
    .p_max = 12.5f,         \
    .v_max = 45.0f,         \
    .t_max = 18.0f,         \
}

// Kp, Kd の範囲はプロトコルで固定
#define DM_KP_MIN  (0.0f)
#define DM_KP_MAX  (500.0f)
#define DM_KD_MIN  (0.0f)
#define DM_KD_MAX  (5.0f)

typedef struct
{
    uint32_t id; // フィードバックのCAN ID (MASTER_ID)
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

typedef struct
{
    uint16_t master_id; // フィードバックのCAN ID
    uint16_t slave_id;  // 指令のCAN ID
    dm_limits_t limits;
} dm_motor_config_t;

/**
 * @brief モーター1台分のインスタンス
 * @note  メンバは直接触らず dm_motor_* 関数を使うこと．
 *        can_bus に登録されるため，can_bus_deinit() まで有効なメモリ (static変数など) に置くこと．
 */
typedef struct
{
    uint16_t master_id;
    uint16_t slave_id;
    dm_limits_t limits;

    // 以下は can_bus の受信タスクが更新する
    portMUX_TYPE lock;
    dm_feedback_t fb;
    TickType_t last_rx_tick;
    uint32_t rx_count;
} dm_motor_t;

/**
 * @brief モーターを初期化し，フィードバックの受信先として can_bus に登録する
 * @note  can_bus_init() の後に呼ぶこと
 */
esp_err_t dm_motor_init(dm_motor_t *motor, const dm_motor_config_t *config);

esp_err_t dm_motor_enable(dm_motor_t *motor, TickType_t ticks_to_wait);
esp_err_t dm_motor_disable(dm_motor_t *motor, TickType_t ticks_to_wait);
esp_err_t dm_motor_pos_init(dm_motor_t *motor, TickType_t ticks_to_wait);
esp_err_t dm_motor_clear_error(dm_motor_t *motor, TickType_t ticks_to_wait);

esp_err_t dm_motor_mit(dm_motor_t *motor, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait);
esp_err_t dm_motor_torque(dm_motor_t *motor, float torque, TickType_t ticks_to_wait);

/**
 * @brief 最新のフィードバックを取得する (どのタスクから呼んでもよい)
 * @param[out] fb  最新のフィードバック
 * @param[out] age 最後に受信してからの経過tick (不要ならNULL)
 * @return ESP_ERR_NOT_FOUND: まだ一度も受信していない
 */
esp_err_t dm_motor_get_feedback(dm_motor_t *motor, dm_feedback_t *fb, TickType_t *age);

void dm_pack_mit_cmd(uint8_t *data, const dm_limits_t *limits, float pos, float vel, float kp, float kd, float torque);

uint32_t dm_float_to_uint(float x, float x_min, float x_max, int bits);
float dm_uint_to_float(uint32_t x, float x_min, float x_max, int bits);

const char *dm_state_to_string(dm_state_t state);

void dm_dump_feedback(const dm_feedback_t *fb);

#ifdef __cplusplus
}
#endif
