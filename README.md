# Damiao

ボードマネージャでesp32 by Espressif Systemsを利用しているなら，バージョン3.3.5以前推奨
ESP32 Arduino library for Damiao DM Series motors.

## Features

- MIT Control
- Torque Control
- Enable / Disable
- Position Initialization
- Feedback Parsing
- TWAI(CAN) Support

## Supported Hardware

- ESP32
- ESP32-S3
- ESP32-C3 (TWAI supported devices)

## Installation

1. Download ZIP
2. Arduino IDE
3. Sketch -> Include Library -> Add .ZIP Library

## Example

```cpp
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

    delay(10);
}
