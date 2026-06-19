#include "damiao.h"
#include "driver/twai.h"
#include <string.h>

#define P_MIN   (-12.5f)
#define P_MAX   ( 12.5f)

#define V_MIN   (-45.0f)
#define V_MAX   ( 45.0f)

#define KP_MIN  (0.0f)
#define KP_MAX  (500.0f)

#define KD_MIN  (0.0f)
#define KD_MAX  (5.0f)

#define T_MIN   (-18.0f)
#define T_MAX   ( 18.0f)

void dm_enable(uint16_t can_id, TickType_t ticks_to_wait)
{
    twai_message_t msg = {
        .identifier = can_id,
        .data_length_code = 8,
        .extd = 0,
    };

    uint8_t enable[8] = DM_ENABLE;

    memcpy(msg.data, enable, 8);

    twai_transmit(&msg, ticks_to_wait);
}

void dm_disable(uint16_t can_id, TickType_t ticks_to_wait)
{
    twai_message_t msg = {
        .identifier = can_id,
        .data_length_code = 8,
    };

    uint8_t disable[8] = DM_DISABLE;

    memcpy(msg.data, disable, 8);

    twai_transmit(&msg, ticks_to_wait);
}

void dm_send_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait)
{
    twai_message_t msg = {
        .identifier = can_id,
        .data_length_code = 8,
        .extd = 0,
    };

    pack_cmd(msg.data,
             0.0f,   // pos
             0.0f,   // vel
             0.0f,   // kp
             0.0f,   // kd
             torque);

    twai_transmit(&msg, ticks_to_wait);
}

esp_err_t dm_receive(uint16_t can_id, dm_feedback_t *fb, TickType_t ticks_to_wait)
{
    twai_message_t rx;
    esp_err_t e = twai_receive(&rx, ticks_to_wait);
    if(e != ESP_OK)return e;
    if(rx.identifier != can_id)return ESP_FAIL;

    uint16_t p_int = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    uint16_t v_int = ((uint16_t)rx.data[3] << 4) | (rx.data[4] >> 4);
    uint16_t t_int = (((uint16_t)rx.data[4] & 0x0F) << 8) | rx.data[5];

    fb->state = rx.data[0] >> 4;
    fb->pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
    fb->vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
    fb->torque = uint_to_float(t_int, T_MIN, T_MAX, 12);
    fb->mos_temp   = rx.data[6];
    fb->motor_temp = rx.data[7];

    return ESP_OK;
}

void pack_cmd(uint8_t *data, float pos, float vel, float kp, float kd, float torque)
{
    uint16_t p_int  = float_to_uint(pos, P_MIN, P_MAX, 16);
    uint16_t v_int  = float_to_uint(vel, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int  = float_to_uint(torque, T_MIN, T_MAX, 12);

    data[0] = p_int >> 8;
    data[1] = p_int;

    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);

    data[4] = kp_int;

    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);

    data[7] = t_int;
}

esp_err_t twai_init(gpio_num_t tx, gpio_num_t rx){
    twai_general_config_t g_config =TWAI_GENERAL_CONFIG_DEFAULT(tx,rx,TWAI_MODE_NORMAL);
    twai_timing_config_t t_config =TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config =TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config,&t_config,&f_config));

    return twai_start();
}

uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x) * span / ((float)((1 << bits) - 1)) + offset;
}