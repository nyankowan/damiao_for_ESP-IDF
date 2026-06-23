#include <Damiao.h>

Damiao motor1(0x01);

void setup()
{
    Serial.begin(115200);

    Damiao::begin(21,22);

    motor1.enable();
}

void loop()
{
    dm_feedback_t fb;

    if(Damiao::receive(&fb))
    {
        Damiao::dumpFeedback(&fb);
    }
    
    delay(10);
}