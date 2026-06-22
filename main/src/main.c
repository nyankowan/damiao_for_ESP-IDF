#include "damiao.h"
#include "freertos/projdefs.h"
#include <stdio.h>

void app_main(void)
{
    twai_init(GPIO_NUM_21,GPIO_NUM_22);

    vTaskDelay(pdMS_TO_TICKS(1000));

    dm_enable(0x01, pdMS_TO_TICKS(10));

    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        printf("send\n");
        dm_send_torque(0x01, 0.5f, pdMS_TO_TICKS(1));

        dm_feedback_t fb;

        if (dm_receive(0x00, &fb, pdMS_TO_TICKS(10)))
        {
            printf(
                "pos=%.3f "
                "vel=%.3f "
                "torque=%.3f "
                "state=%u\n",
                fb.pos,
                fb.vel,
                fb.torque,
                fb.state);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}