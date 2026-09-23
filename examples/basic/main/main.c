#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_bus.h"
#include "damiao.h"

#define MAIN_TAG "main"

#define TWAI_TX GPIO_NUM_21
#define TWAI_RX GPIO_NUM_22

#define CONTROL_PERIOD_MS 10
#define MONITOR_PERIOD_MS 100 // 表示とエラー処理の周期
#define FEEDBACK_TIMEOUT  pdMS_TO_TICKS(100)

// MASTER_ID と SLAVE_ID の対ごとにインスタンスを作る
static dm_motor_t s_motor1;
static dm_motor_t s_motor2;

static const dm_motor_config_t s_motor1_config = {
    .master_id = 0x11,
    .slave_id  = 0x01,
    .limits    = DM_LIMITS_DEFAULT,
};
static const dm_motor_config_t s_motor2_config = {
    .master_id = 0x12,
    .slave_id  = 0x02,
    .limits    = DM_LIMITS_DEFAULT,
};

static void on_unknown_frame(const can_frame_t *frame, void *ctx);
static void error_handle_dm(dm_motor_t *motor, const dm_feedback_t *fb);
static void main_loop(void);

void app_main(void)
{
    const can_bus_config_t bus_config = CAN_BUS_DEFAULT_CONFIG(TWAI_TX, TWAI_RX);
    ESP_ERROR_CHECK(can_bus_init(&bus_config));

    ESP_ERROR_CHECK(dm_motor_init(&s_motor1, &s_motor1_config));
    ESP_ERROR_CHECK(dm_motor_init(&s_motor2, &s_motor2_config));
    // モーター以外のフレーム (他のデバイスなど) はここに届く
    ESP_ERROR_CHECK(can_bus_set_default_handler(on_unknown_frame, NULL));

    vTaskDelay(pdMS_TO_TICKS(1000));

    dm_motor_enable(&s_motor1, pdMS_TO_TICKS(10));
    dm_motor_enable(&s_motor2, pdMS_TO_TICKS(10));

    vTaskDelay(pdMS_TO_TICKS(100));

    //never return
    main_loop();
}

static void main_loop(void)
{
    dm_motor_t *motors[] = { &s_motor1, &s_motor2 };
    TickType_t wake = xTaskGetTickCount();
    uint32_t count = 0;

    while (1)
    {
        dm_motor_torque(&s_motor1, 0.5f, 0);
        dm_motor_mit(&s_motor2, 0.0f/*position*/, 0.0f/*velocity*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/, 0);

        // 受信はcan_busの受信タスクが行うので，ここでは最新値を読むだけ
        if (++count % (MONITOR_PERIOD_MS / CONTROL_PERIOD_MS) == 0) {
            for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
                dm_feedback_t fb;
                TickType_t age;
                if (dm_motor_get_feedback(motors[i], &fb, &age) != ESP_OK || age > FEEDBACK_TIMEOUT) {
                    ESP_LOGW(MAIN_TAG, "motor%d: no feedback", (int)i + 1);
                    continue;
                }
                dm_dump_feedback(&fb);
                error_handle_dm(motors[i], &fb);
            }
        }

        xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void on_unknown_frame(const can_frame_t *frame, void *ctx)
{
    // can_busの受信タスクから呼ばれるので，重い処理はしない
    ESP_LOGW(MAIN_TAG, "unknown frame id=0x%03" PRIX32 " len=%d", frame->id, frame->len);
}

static void error_handle_dm(dm_motor_t *motor, const dm_feedback_t *fb)
{
    switch(fb->state){
        case DM_STATE_ENABLE:
            break;

        case DM_STATE_DISABLE:
            dm_motor_enable(motor, 0);
            break;

        case DM_STATE_MOS_OVER_TEMP:
            if(fb->mos_temp < 40){
                dm_motor_clear_error(motor, 0);
            }
            break;

        case DM_STATE_MOTOR_OVER_TEMP:
            if(fb->motor_temp < 40){
                dm_motor_clear_error(motor, 0);
            }
            break;

        case DM_STATE_CAN_TIMEOUT:
            dm_motor_clear_error(motor, 0);
            break;

        case DM_STATE_UNDERVOLTAGE:
        case DM_STATE_OVERVOLTAGE:
        case DM_STATE_OVERCURRENT:
        case DM_STATE_OVERLOAD:
        default:
            ESP_LOGE(MAIN_TAG, "id=0x%02" PRIX32 " %s", fb->id, dm_state_to_string(fb->state));
            break;
    }
}
