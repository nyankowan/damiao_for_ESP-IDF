#include <Damiao.h>

Damiao motor(0x01);

void setup()
{
    Serial.begin(115200);

    Damiao::begin(21,22);

    motor.enable();
}

void loop()
{
    motor.sendTorque(1.0f);

    dm_feedback_t fb;

    if(Damiao::receive(&fb))
    {
        Damiao::dumpFeedback(&fb);
    }
    
    delay(10);
}