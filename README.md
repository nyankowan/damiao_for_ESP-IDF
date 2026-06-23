# DAMIAO for ESP-IDF
MITモードに対応\
IDF-version: v5.4.4(おそらくv6.xでも使用可能)\
ESP board: ESP32-WROVER-KIT-3.3v

## for Arduino IDE
ほぼChatGPT製ですが，for_arduinoブランチからダウンロードすればArduino IDEでも使用可能です.

# コードの概要
twai_initによりcanbusを有効化．\
can通信により，モーターに有効化するデータを送る．
```c:main.c
 #define TYPE 0
        #if TYPE == 0
            dm_transmit_torque(SLAVE_ID, 0.5f, pdMS_TO_TICKS(1));
        #elif TYPE == 1
            dm_transmit_mit(SLAVE_ID, 0.0f/*position*/, 0.0f/*velocity*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/ ,pdMS_TO_TICKS(1));
        #endif
```
TYPEを0にすると，トルクのみの信号を送る．mitのtorque以外の部分を0にしたときと同じ挙動．\
TYPEを1にすると，MIT形式で送る．
### MIT 制御式
$$
\tau = K_p(q_d-q) + K_d(\dot q_d-\dot q) + \tau_{ff}
$$

## フィードバック
canからフィードバックを受け取る\
内容は`dm_feedback_t`として受け取り，モーターの\
ID，角度(rad)，角速度(rad/s)，トルク(Nm)，状態(enable,disable,undervoltageなど)，モタドラMOSFETの温度(℃)，モーター温度(℃)\
がわかる．

モーターの状態がとりうる値は`main/inc/damiao.h`の`dm_state_t`を参照するといい．
```c:damio.h
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
```
