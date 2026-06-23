#pragma once

#include <Arduino.h>
#include "driver/twai.h"

#define DM_ENABLE      {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}
#define DM_DISABLE     {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}
#define DM_POS_INIT    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}
#define DM_CLEAR_ERROR {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}

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

typedef enum
{
    DM_STATE_DISABLE         = 0x0,
    DM_STATE_ENABLE          = 0x1,

    DM_STATE_OVERVOLTAGE     = 0x8,
    DM_STATE_UNDERVOLTAGE    = 0x9,
    DM_STATE_OVERCURRENT     = 0xA,

    DM_STATE_MOS_OVER_TEMP   = 0xB,
    DM_STATE_MOTOR_OVER_TEMP = 0xC,

    DM_STATE_CAN_TIMEOUT     = 0xD,
    DM_STATE_OVERLOAD        = 0xE

} dm_state_t;

typedef struct
{
    int16_t id;

    float pos;
    float vel;
    float torque;

    dm_state_t state;

    uint8_t motor_temp;
    uint8_t mos_temp;

} dm_feedback_t;

class Damiao
{
public:

    Damiao(uint16_t can_id);

    static bool begin(
        int tx_pin,
        int rx_pin,
        uint32_t baud = 1000000
    );

    bool enable();
    bool disable();

    bool posInit();

    bool clearError();

    bool sendMIT(
        float pos,
        float vel,
        float kp,
        float kd,
        float torque
    );

    bool sendTorque(float torque);

    static bool receive(dm_feedback_t *fb);

    static void dumpFeedback(dm_feedback_t *fb);

    static const char *stateToString(dm_state_t state);

private:

    uint16_t _id;

    static esp_err_t transmit(
        uint16_t can_id,
        uint8_t *data
    );

    static void packCmd(
        uint8_t *data,
        float pos,
        float vel,
        float kp,
        float kd,
        float torque
    );

    static uint32_t floatToUint(
        float x,
        float x_min,
        float x_max,
        int bits
    );

    static float uintToFloat(
        uint32_t x,
        float x_min,
        float x_max,
        int bits
    );
};