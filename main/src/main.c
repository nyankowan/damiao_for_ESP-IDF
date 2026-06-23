#include "damiao.h"
#include "freertos/projdefs.h"
#include <stdio.h>
#include "driver/twai.h"
#include "esp_log.h"

#define MAIN_TAG "main"

#define TWAI_TX GPIO_NUM_21
#define TWAI_RX GPIO_NUM_22 

#define MASTER_ID 0x12
#define SLAVE_ID 0x02

void main_loop();
void error_handle_dm(dm_feedback_t *fb);

void app_main(void)
{
    if(twai_init(TWAI_TX,TWAI_RX) != ESP_OK){
        ESP_LOGE(MAIN_TAG, "twai_init failed");
        exit(-1);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    dm_enable(SLAVE_ID, pdMS_TO_TICKS(10));

    vTaskDelay(pdMS_TO_TICKS(100));

    //never return
    main_loop();
}

void main_loop()
{
    while (1)
    {
        #define TYPE 0
        #if TYPE == 0
            dm_transmit_torque(SLAVE_ID, 0.5f, pdMS_TO_TICKS(1));
        #elif TYPE == 1
            dm_transmit_mit(SLAVE_ID, 0.0f/*position*/, 0.0f/*velocity*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/ ,pdMS_TO_TICKS(1));
        #endif

        dm_feedback_t fb;
        if (dm_receive(&fb, pdMS_TO_TICKS(10)) == ESP_OK){
            if(fb.id == MASTER_ID) dump_dm_feedback(&fb);
            error_handle_dm(&fb);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void error_handle_dm(dm_feedback_t *fb)
{
    ESP_LOGI(MAIN_TAG, "%s", dm_state_to_string(fb->state));
    switch(fb->state){
        case DM_STATE_DISABLE:
            dm_enable(SLAVE_ID, pdMS_TO_TICKS(10));
            break;
        
        case DM_STATE_MOS_OVER_TEMP:
            if(fb->mos_temp < 40){
                dm_clear_error(SLAVE_ID, pdMS_TO_TICKS(10));
            }
            break;

        case DM_STATE_MOTOR_OVER_TEMP:
            if(fb->motor_temp < 40){
                dm_clear_error(SLAVE_ID, pdMS_TO_TICKS(10));
            }
            break;

        case DM_STATE_CAN_TIMEOUT:
            dm_clear_error(SLAVE_ID, pdMS_TO_TICKS(10));
            break;

        case DM_STATE_UNDERVOLTAGE:
        case DM_STATE_OVERVOLTAGE:
        case DM_STATE_OVERCURRENT:
        case DM_STATE_OVERLOAD:
        default:
            break;
    }
    
}