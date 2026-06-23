#include "damiao.h"
#include "driver/twai.h"
#include "esp_log.h"

#define DAMIAO_TAG "damiao"

esp_err_t dm_transmit(uint16_t can_id, uint8_t *data ,TickType_t ticks_to_wait)
{
    twai_message_t msg = {
        .identifier = can_id,
        .data_length_code = 8,
        .extd = 0,
    };

    for(int i = 0; i < 8; i++){
        msg.data[i] = data[i];
    }

    return twai_transmit(&msg, ticks_to_wait);
}

esp_err_t dm_transmit_mit(uint16_t can_id, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait)
{
    uint8_t data[8];
    pack_cmd(data, pos, vel, kp, kd, torque);

    return dm_transmit(can_id, data, ticks_to_wait);
}

esp_err_t dm_enable(uint16_t can_id, TickType_t ticks_to_wait)
{
    uint8_t enable[8] = DM_ENABLE;
    return dm_transmit(can_id, enable ,ticks_to_wait);
}

esp_err_t dm_pos_init(uint16_t can_id, TickType_t ticks_to_wait)
{
    uint8_t disable[8] = DM_POS_INIT;
    return dm_transmit(can_id, disable ,ticks_to_wait);
}

esp_err_t dm_clear_error(uint16_t can_id, TickType_t ticks_to_wait)
{
    uint8_t disable[8] = DM_CLEAR_ERROR;
    return dm_transmit(can_id, disable ,ticks_to_wait);
}

esp_err_t dm_disable(uint16_t can_id, TickType_t ticks_to_wait)
{
    uint8_t disable[8] = DM_DISABLE;
    return dm_transmit(can_id, disable ,ticks_to_wait);
}

esp_err_t dm_transmit_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait)
{
    uint8_t data[8];
    pack_cmd(data,
             0.0f,   // pos
             0.0f,   // vel
             0.0f,   // kp
             0.0f,   // kd
             torque);

    return dm_transmit(can_id, data ,ticks_to_wait);
}

esp_err_t dm_receive(dm_feedback_t *fb, TickType_t ticks_to_wait)
{
    twai_message_t rx;
    esp_err_t e = twai_receive(&rx, ticks_to_wait);
    if(e != ESP_OK){
        ESP_LOGV(DAMIAO_TAG, "receive timeout");
        return e;
    }
    fb->id = rx.identifier;
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

void dump_dm_feedback(dm_feedback_t *fb){
    printf(
            "rx id=0x%02X "
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

void dump_twai_status(){
    twai_status_info_t status;
    twai_get_status_info(&status);

    printf(
        "tx=%ld rx=%ld tx_failed=%ld bus_error=%ld\n",
        status.msgs_to_tx,
        status.msgs_to_rx,
        status.tx_failed_count,
        status.bus_error_count
    );
}