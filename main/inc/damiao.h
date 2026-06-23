#include "freertos/FreeRTOS.h"
#include "soc/gpio_num.h"

#define DM_ENABLE {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}
#define DM_DISABLE {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}
#define DM_POS_INIT {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}
#define DM_CLEAR_ERROR {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}

//default configure
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
    int16_t id;
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

esp_err_t dm_transmit(uint16_t can_id, uint8_t *data ,TickType_t ticks_to_wait);
esp_err_t dm_transmit_mit(uint16_t can_id, float pos, float vel, float kp, float kd, float torque, TickType_t ticks_to_wait);
esp_err_t dm_transmit_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait);

esp_err_t dm_enable(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_disable(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_pos_init(uint16_t can_id, TickType_t ticks_to_wait);
esp_err_t dm_clear_error(uint16_t can_id, TickType_t ticks_to_wait);

esp_err_t dm_receive(dm_feedback_t *fb, TickType_t ticks_to_wait);

esp_err_t twai_init(gpio_num_t tx, gpio_num_t rx);

void pack_cmd(uint8_t *data, float pos, float vel, float kp, float kd, float torque);

uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(uint32_t x, float x_min, float x_max, int bits);

const char *dm_state_to_string(dm_state_t state);

void dump_dm_feedback(dm_feedback_t *fb);
void dump_twai_status();