#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "damiao.h"
#include "esp_check.h"
#include "freertos/task.h"

#define DAMIAO_TAG "damiao"

#define DM_FRAME_LEN 8

#define DM_CMD_ENABLE      {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}
#define DM_CMD_DISABLE     {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}
#define DM_CMD_POS_INIT    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}
#define DM_CMD_CLEAR_ERROR {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}

static void dm_decode_feedback(const uint8_t *data, const dm_limits_t *lim, dm_feedback_t *fb)
{
    uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t t_int = (((uint16_t)data[4] & 0x0F) << 8) | data[5];

    fb->state = data[0] >> 4;
    fb->pos = dm_uint_to_float(p_int, -lim->p_max, lim->p_max, 16);
    fb->vel = dm_uint_to_float(v_int, -lim->v_max, lim->v_max, 12);
    fb->torque = dm_uint_to_float(t_int, -lim->t_max, lim->t_max, 12);
    fb->mos_temp   = data[6];
    fb->motor_temp = data[7];
}

// can_bus の受信タスクから呼ばれる
static void dm_motor_on_frame(const can_frame_t *frame, void *ctx)
{
    dm_motor_t *motor = ctx;

    if (frame->rtr || frame->len < DM_FRAME_LEN) {
        return;
    }
    // data[0]の下位4bitは送信元モーターのSLAVE_ID下位4bit．
    // 複数のモーターが同じMASTER_IDを使っている場合はここで区別する
    if ((frame->data[0] & 0x0F) != (motor->slave_id & 0x0F)) {
        return;
    }

    dm_feedback_t fb = { .id = frame->id };
    dm_decode_feedback(frame->data, &motor->limits, &fb);

    portENTER_CRITICAL(&motor->lock);
    motor->fb = fb;
    motor->last_rx_tick = xTaskGetTickCount();
    motor->rx_count++;
    portEXIT_CRITICAL(&motor->lock);
}

esp_err_t dm_motor_init(dm_motor_t *motor, const dm_motor_config_t *config)
{
    ESP_RETURN_ON_FALSE(motor && config, ESP_ERR_INVALID_ARG, DAMIAO_TAG, "invalid arg");

    *motor = (dm_motor_t) {
        .master_id = config->master_id,
        .slave_id = config->slave_id,
        .limits = config->limits,
    };
    portMUX_INITIALIZE(&motor->lock);

    return can_bus_register(config->master_id, CAN_BUS_STD_ID_MASK, false, dm_motor_on_frame, motor);
}

static esp_err_t dm_motor_transmit(dm_motor_t *motor, const uint8_t *data, TickType_t ticks_to_wait)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, DAMIAO_TAG, "motor is NULL");

    can_frame_t frame = {
        .id = motor->slave_id,
        .len = DM_FRAME_LEN,
    };
    memcpy(frame.data, data, DM_FRAME_LEN);
    return can_bus_transmit(&frame, ticks_to_wait);
}

esp_err_t dm_motor_enable(dm_motor_t *motor, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_ENABLE;
    return dm_motor_transmit(motor, cmd, ticks_to_wait);
}

esp_err_t dm_motor_disable(dm_motor_t *motor, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_DISABLE;
    return dm_motor_transmit(motor, cmd, ticks_to_wait);
}

esp_err_t dm_motor_pos_init(dm_motor_t *motor, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_POS_INIT;
    return dm_motor_transmit(motor, cmd, ticks_to_wait);
}

esp_err_t dm_motor_clear_error(dm_motor_t *motor, TickType_t ticks_to_wait)
{
    const uint8_t cmd[DM_FRAME_LEN] = DM_CMD_CLEAR_ERROR;
    return dm_motor_transmit(motor, cmd, ticks_to_wait);
}

esp_err_t dm_motor_mit(dm_motor_t *motor, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, DAMIAO_TAG, "motor is NULL");

    uint8_t data[DM_FRAME_LEN];
    dm_pack_mit_cmd(data, &motor->limits, pos, vel, kp, kd, torque);
    return dm_motor_transmit(motor, data, ticks_to_wait);
}

esp_err_t dm_motor_torque(dm_motor_t *motor, float torque, TickType_t ticks_to_wait)
{
    return dm_motor_mit(motor,
                        0.0f,   // pos
                        0.0f,   // vel
                        0.0f,   // kp
                        0.0f,   // kd
                        torque,
                        ticks_to_wait);
}

esp_err_t dm_motor_get_feedback(dm_motor_t *motor, dm_feedback_t *fb, TickType_t *age)
{
    ESP_RETURN_ON_FALSE(motor && fb, ESP_ERR_INVALID_ARG, DAMIAO_TAG, "invalid arg");

    portENTER_CRITICAL(&motor->lock);
    *fb = motor->fb;
    TickType_t last_rx_tick = motor->last_rx_tick;
    uint32_t rx_count = motor->rx_count;
    portEXIT_CRITICAL(&motor->lock);

    if (age) {
        *age = xTaskGetTickCount() - last_rx_tick;
    }
    return rx_count > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void dm_pack_mit_cmd(uint8_t *data, const dm_limits_t *limits, float pos, float vel, float kp, float kd, float torque)
{
    uint16_t p_int  = dm_float_to_uint(pos, -limits->p_max, limits->p_max, 16);
    uint16_t v_int  = dm_float_to_uint(vel, -limits->v_max, limits->v_max, 12);
    uint16_t kp_int = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t t_int  = dm_float_to_uint(torque, -limits->t_max, limits->t_max, 12);

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
