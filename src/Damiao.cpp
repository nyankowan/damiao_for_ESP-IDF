#include "Damiao.h"

Damiao::Damiao(uint16_t can_id)
{
    _id = can_id;
}

bool Damiao::begin(
    int tx_pin,
    int rx_pin,
    uint32_t baud
)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)tx_pin,
            (gpio_num_t)rx_pin,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config;

    switch(baud)
    {
        case 1000000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;

        case 500000:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;

        case 250000:
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            break;

        default:
            return false;
    }

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if(
        twai_driver_install(
            &g_config,
            &t_config,
            &f_config
        ) != ESP_OK
    )
    {
        return false;
    }

    return twai_start() == ESP_OK;
}

esp_err_t Damiao::transmit(
    uint16_t can_id,
    uint8_t *data
)
{
    twai_message_t msg = {};

    msg.identifier = can_id;
    msg.data_length_code = 8;
    msg.extd = 0;

    memcpy(msg.data,data,8);

    return twai_transmit(
        &msg,
        pdMS_TO_TICKS(10)
    );
}

bool Damiao::enable()
{
    uint8_t cmd[8] = DM_ENABLE;

    return transmit(_id,cmd) == ESP_OK;
}

bool Damiao::disable()
{
    uint8_t cmd[8] = DM_DISABLE;

    return transmit(_id,cmd) == ESP_OK;
}

bool Damiao::posInit()
{
    uint8_t cmd[8] = DM_POS_INIT;

    return transmit(_id,cmd) == ESP_OK;
}

bool Damiao::clearError()
{
    uint8_t cmd[8] = DM_CLEAR_ERROR;

    return transmit(_id,cmd) == ESP_OK;
}

bool Damiao::sendMIT(
    float pos,
    float vel,
    float kp,
    float kd,
    float torque
)
{
    uint8_t data[8];

    packCmd(
        data,
        pos,
        vel,
        kp,
        kd,
        torque
    );

    return transmit(_id,data) == ESP_OK;
}

bool Damiao::sendTorque(float torque)
{
    return sendMIT(
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        torque
    );
}

bool Damiao::receive(dm_feedback_t *fb)
{
    twai_message_t rx;

    if(
        twai_receive(
            &rx,
            0
        ) != ESP_OK
    )
    {
        return false;
    }

    fb->id = rx.identifier;

    uint16_t p_int =
        ((uint16_t)rx.data[1] << 8)
        | rx.data[2];

    uint16_t v_int =
        ((uint16_t)rx.data[3] << 4)
        | (rx.data[4] >> 4);

    uint16_t t_int =
        (((uint16_t)rx.data[4] & 0x0F) << 8)
        | rx.data[5];

    fb->state =
        (dm_state_t)(rx.data[0] >> 4);

    fb->pos =
        uintToFloat(
            p_int,
            P_MIN,
            P_MAX,
            16
        );

    fb->vel =
        uintToFloat(
            v_int,
            V_MIN,
            V_MAX,
            12
        );

    fb->torque =
        uintToFloat(
            t_int,
            T_MIN,
            T_MAX,
            12
        );

    fb->mos_temp = rx.data[6];
    fb->motor_temp = rx.data[7];

    return true;
}

void Damiao::packCmd(
    uint8_t *data,
    float pos,
    float vel,
    float kp,
    float kd,
    float torque
)
{
    uint16_t p_int =
        floatToUint(pos,P_MIN,P_MAX,16);

    uint16_t v_int =
        floatToUint(vel,V_MIN,V_MAX,12);

    uint16_t kp_int =
        floatToUint(kp,KP_MIN,KP_MAX,12);

    uint16_t kd_int =
        floatToUint(kd,KD_MIN,KD_MAX,12);

    uint16_t t_int =
        floatToUint(torque,T_MIN,T_MAX,12);

    data[0] = p_int >> 8;
    data[1] = p_int;

    data[2] = v_int >> 4;

    data[3] =
        ((v_int & 0x0F) << 4)
        | (kp_int >> 8);

    data[4] = kp_int;

    data[5] = kd_int >> 4;

    data[6] =
        ((kd_int & 0x0F) << 4)
        | (t_int >> 8);

    data[7] = t_int;
}

uint32_t Damiao::floatToUint(
    float x,
    float x_min,
    float x_max,
    int bits
)
{
    float span = x_max - x_min;

    return
        (uint32_t)(
            (x - x_min)
            * ((1 << bits) - 1)
            / span
        );
}

float Damiao::uintToFloat(
    uint32_t x,
    float x_min,
    float x_max,
    int bits
)
{
    float span = x_max - x_min;

    return
        ((float)x)
        * span
        / ((1 << bits) - 1)
        + x_min;
}

const char *Damiao::stateToString(
    dm_state_t state
)
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

void Damiao::dumpFeedback(
    dm_feedback_t *fb
)
{
    Serial.printf(
        "id=%d "
        "pos=%.3f "
        "vel=%.3f "
        "torque=%.3f "
        "state=%s "
        "mos=%d "
        "motor=%d\n",
        fb->id,
        fb->pos,
        fb->vel,
        fb->torque,
        stateToString(fb->state),
        fb->mos_temp,
        fb->motor_temp
    );
}