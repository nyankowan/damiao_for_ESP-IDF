#include "freertos/FreeRTOS.h"
#include "soc/gpio_num.h"

#define DM_ENABLE {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}
#define DM_DISABLE {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}

typedef struct
{
    float pos;
    float vel;
    float torque;

    uint8_t state;
    uint8_t motor_temp;
    uint8_t mos_temp;

} dm_feedback_t;

void dm_enable(uint16_t can_id, TickType_t ticks_to_wait);
void dm_disable(uint16_t can_id, TickType_t ticks_to_wait);
void dm_send_torque(uint16_t can_id, float torque, TickType_t ticks_to_wait);
esp_err_t dm_receive(uint16_t can_id, dm_feedback_t *fb, TickType_t ticks_to_wait);
esp_err_t twai_init(gpio_num_t tx, gpio_num_t rx);

void pack_cmd(uint8_t *data,
                     float pos,
                     float vel,
                     float kp,
                     float kd,
                     float torque);

uint32_t float_to_uint(float x,
                              float x_min,
                              float x_max,
                              int bits);

float uint_to_float(uint32_t x,
                           float x_min,
                           float x_max,
                           int bits);