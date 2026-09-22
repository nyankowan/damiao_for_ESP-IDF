# DAMIAO for ESP-IDF
DAMIAO製モーターをESP32のTWAI(CAN)から制御するESP-IDFコンポーネント．MITモードに対応\
IDF-version: v5.5 以降 (v6.0 で動作確認)\
ESP board: ESP32-WROVER-KIT-3.3v

TWAIは ESP-IDF v5.5 で追加された新ドライバ (`esp_driver_twai` / `esp_twai.h`) を使用しています．
v6.0 で非推奨となった旧ドライバ (`driver/twai.h`) には依存しません．

## for Arduino IDE
ほぼChatGPT製ですが，for_arduinoブランチからダウンロードすればArduino IDEでも使用可能です.

# ディレクトリ構成
```
damiao_for_ESP-IDF/
├── components/
│   └── damiao/                  ← コンポーネント本体 (これを配布・利用する)
│       ├── CMakeLists.txt
│       ├── idf_component.yml    ← コンポーネントマネージャ用マニフェスト
│       ├── Kconfig              ← menuconfig の "DAMIAO motor driver"
│       ├── include/damiao.h
│       └── src/damiao.c
└── examples/
    └── basic/                   ← サンプルプロジェクト
        ├── CMakeLists.txt
        ├── sdkconfig.defaults
        └── main/
            ├── CMakeLists.txt
            ├── idf_component.yml
            └── main.c
```

# 使い方
## 自分のプロジェクトに追加する
### コンポーネントマネージャ (git) を使う (推奨)
`main/idf_component.yml` に以下を追加する (無ければ作成する)．ビルド時に自動でダウンロードされる．
```yaml
dependencies:
  nyankowan/damiao:
    git: https://github.com/nyankowan/damiao_for_ESP-IDF.git
    path: components/damiao
    version: develop   # ブランチ/タグ/コミット
```
または
```sh
idf.py add-dependency --git https://github.com/nyankowan/damiao_for_ESP-IDF.git --git-ref develop --git-path components/damiao nyankowan/damiao
```

### 手動でコピーする
`components/damiao/` ディレクトリを自分のプロジェクトの `components/damiao/` にコピーする．

`main` 以外のコンポーネントから使う場合は，`idf_component_register()` の `REQUIRES` にコンポーネント名
(コンポーネントマネージャ経由なら `nyankowan__damiao`，手動でコピーした場合は `damiao`) を追加する．

## サンプルをビルドする
```sh
cd examples/basic
idf.py set-target esp32
idf.py build flash monitor
```

## 設定 (menuconfig)
`idf.py menuconfig` → `Component config` → `DAMIAO motor driver`

| 項目 | デフォルト | 説明 |
|---|---|---|
| `CONFIG_DM_TWAI_BITRATE` | 1000000 | CANのビットレート |
| `CONFIG_DM_TWAI_TX_QUEUE_LEN` | 8 | 送信待ちにできるフレーム数 |
| `CONFIG_DM_TWAI_RX_QUEUE_LEN` | 16 | `dm_receive()` 前に保持できる受信フレーム数 |

位置・速度・ゲイン・トルクの範囲 (`DM_P_MAX`, `DM_V_MAX`, `DM_T_MAX` など) はDM4310のデフォルト値．
モーター側の設定と異なる場合は，プロジェクトの `CMakeLists.txt` でコンパイルオプションとして上書きする．
```cmake
# <project>/CMakeLists.txt
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(COMPILE_DEFINITIONS "DM_T_MAX=10.0f" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "DM_T_MIN=-10.0f" APPEND)
project(my_project)
```

# コードの概要
`dm_twai_init`によりcanbusを有効化．\
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

モーターの状態がとりうる値は`components/damiao/include/damiao.h`の`dm_state_t`を参照するといい．
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

# 旧バージョンからの変更点
| 旧 | 新 |
|---|---|
| `twai_init(tx, rx)` | `dm_twai_init(tx, rx)` |
| `dump_dm_feedback()` | `dm_dump_feedback()` |
| `dump_twai_status()` | `dm_dump_twai_status()` |
| `pack_cmd()` | `dm_pack_mit_cmd()` |
| `float_to_uint()` / `uint_to_float()` | `dm_float_to_uint()` / `dm_uint_to_float()` |
| `P_MIN`, `T_MAX` など | `DM_P_MIN`, `DM_T_MAX` など |
| `DM_ENABLE` などのコマンドマクロ | 内部化 (`dm_enable()` などを使用) |

- 送受信関数はタスクから呼ぶこと (ISRからは不可)．
- `dm_float_to_uint()` は範囲外の値を飽和させるようになった (以前は隣のフィールドを壊していた)．
- バスオフからの復帰は `dm_twai_recover()`，TWAIノードを直接操作したい場合は `dm_twai_get_node()` を使う．
