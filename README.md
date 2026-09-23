# DAMIAO for ESP-IDF
DAMIAO製モーターをESP32のTWAI(CAN)から制御するESP-IDFコンポーネント．MITモードに対応．複数モーターの同時制御に対応\
IDF-version: v5.5 以降 (v6.0 で動作確認)\
ESP board: ESP32-WROVER-KIT-3.3v

TWAIは ESP-IDF v5.5 で追加された新ドライバ (`esp_driver_twai` / `esp_twai.h`) を使用しています．
v6.0 で非推奨となった旧ドライバ (`driver/twai.h`) には依存しません．

## for Arduino IDE
ほぼChatGPT製ですが，for_arduinoブランチからダウンロードすればArduino IDEでも使用可能です.

# 構成
2つのコンポーネントからなる．
```
main ──→ damiao ──→ can_bus ──→ esp_driver_twai
  └──────────────────→ can_bus  (他のデバイスや未知フレームの受け取りを登録する場合)
```
- **can_bus**: TWAIの送受信を一手に引き受け，受信フレームをCAN IDごとに登録されたハンドラへ配送する．
  damiaoに依存しないので，同じバスにつながる他のデバイス (センサなど) のドライバもこの上に載せられる．
  どのハンドラにも一致しなかったフレームはデフォルトハンドラに届くので，情報が捨てられることはない．
- **damiao**: モーター1台 (MASTER_ID と SLAVE_ID の対) ごとに `dm_motor_t` のインスタンスを作って使う．
  `dm_motor_init()` で自分のMASTER_IDを can_bus に登録し，フィードバックは受信タスクが自動で更新する．

## ディレクトリ構成
```
damiao_for_ESP-IDF/
├── components/
│   ├── can_bus/                 ← CANバス共通層 (受信フレームの振り分け)
│   │   ├── CMakeLists.txt
│   │   ├── idf_component.yml
│   │   ├── Kconfig              ← menuconfig の "CAN bus dispatcher"
│   │   ├── include/can_bus.h
│   │   └── src/can_bus.c
│   └── damiao/                  ← モータードライバ (can_bus に依存)
│       ├── CMakeLists.txt
│       ├── idf_component.yml    ← コンポーネントマネージャ用マニフェスト
│       ├── include/damiao.h
│       └── src/damiao.c
└── examples/
    └── basic/                   ← サンプルプロジェクト
        ├── .vscode/             ← VS Code (ESP-IDF拡張) 用設定
        ├── CMakeLists.txt
        ├── sdkconfig.defaults
        └── main/
            ├── CMakeLists.txt
            ├── idf_component.yml
            └── main.c
```

# 使い方
## 自分のプロジェクトに組み込む
必要なもの
- ESP-IDF v5.5 以降 (v6 推奨)
- git (コンポーネントマネージャがダウンロードに使用する)

以下のコマンドはESP-IDFの環境が有効なターミナル (Windowsなら「ESP-IDF PowerShell/CMD」，VS Codeなら `ESP-IDF: Open ESP-IDF Terminal`) で実行する．

### 方法1: コンポーネントマネージャで取り込む (推奨)
ビルド時にGitHubから自動でダウンロードされる．ファイルのコピーは不要．

#### 1. プロジェクトを用意する
既存のプロジェクトに組み込む場合は不要．
```sh
idf.py create-project my_robot
cd my_robot
idf.py set-target esp32
```

#### 2. 依存を追加する
プロジェクトのルートで実行する．
```sh
idf.py add-dependency --git https://github.com/nyankowan/damiao_for_ESP-IDF.git --git-path components/damiao --git-ref develop nyankowan/damiao
```
`main/idf_component.yml` が作成 (または追記) される．手で書いてもよい．
依存している can_bus も自動で取り込まれる．
```yaml
dependencies:
  nyankowan/damiao:
    git: https://github.com/nyankowan/damiao_for_ESP-IDF.git
    path: components/damiao   # リポジトリ内のコンポーネントの場所
    version: develop          # ブランチ/タグ/コミット
```
> `version` にブランチ名を指定すると，`idf.py update-dependencies` を実行したときにそのブランチの最新が取り込まれる．
> 動作を固定したい場合はタグ (例: `v1.0.0`) やコミットハッシュを指定する．

#### 3. コードを書く
`main` コンポーネントからは `#include "damiao.h"` するだけで使える (`main/CMakeLists.txt` の変更は不要)．
```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "damiao.h"

static dm_motor_t s_motor1, s_motor2; // can_busに登録されるのでstaticに置く

void app_main(void)
{
    const can_bus_config_t bus_config = CAN_BUS_DEFAULT_CONFIG(GPIO_NUM_21, GPIO_NUM_22); // TX, RX
    ESP_ERROR_CHECK(can_bus_init(&bus_config));

    // MASTER_ID と SLAVE_ID の対ごとにインスタンスを作る
    const dm_motor_config_t m1 = { .master_id = 0x11, .slave_id = 0x01, .limits = DM_LIMITS_DEFAULT };
    const dm_motor_config_t m2 = { .master_id = 0x12, .slave_id = 0x02, .limits = DM_LIMITS_DEFAULT };
    ESP_ERROR_CHECK(dm_motor_init(&s_motor1, &m1));
    ESP_ERROR_CHECK(dm_motor_init(&s_motor2, &m2));

    dm_motor_enable(&s_motor1, pdMS_TO_TICKS(10));
    dm_motor_enable(&s_motor2, pdMS_TO_TICKS(10));

    while (1) {
        dm_motor_mit(&s_motor1, 0.0f/*pos*/, 0.0f/*vel*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/, 0);
        dm_motor_torque(&s_motor2, 0.5f, 0);

        // 受信はcan_busの受信タスクが行うので，最新値を読むだけ
        dm_feedback_t fb;
        if (dm_motor_get_feedback(&s_motor1, &fb, NULL) == ESP_OK) {
            dm_dump_feedback(&fb);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

#### 4. ビルド・書き込み
```sh
idf.py build flash monitor
```
初回ビルド時に以下が自動で作られる．
```
my_robot/
├── managed_components/
│   ├── nyankowan__damiao/                 ← ダウンロードされたコンポーネント (編集しない．更新時に上書きされる)
│   └── nyankowan__can_bus/
└── dependencies.lock                      ← 解決したバージョンの記録
```
`managed_components/` は `.gitignore` に追加してよい．`dependencies.lock` はコミットしておくと全員が同じバージョンでビルドできる．

#### 更新する
```sh
idf.py update-dependencies
```
`dependencies.lock` がある間はビルドしても自動では更新されないため，最新を取り込むときはこのコマンドを実行する．

### 方法2: 手動でコピーする
ネットワークを使わずにビルドしたい場合や，コンポーネントをプロジェクト内で改造したい場合．このリポジトリの `components/can_bus/` と `components/damiao/` をフォルダごと，自分のプロジェクトの `components/` にコピーする．
```
my_robot/
├── CMakeLists.txt
├── components/
│   ├── can_bus/       ← コピーしたもの
│   └── damiao/        ← コピーしたもの
└── main/
    └── idf_component.yml
```
damiao の `idf_component.yml` は can_bus をGitHubから取得するよう書かれているため，
`main/idf_component.yml` でコピーした can_bus を使うよう指定する (これがないとGitHubからダウンロードしようとする)．
```yaml
dependencies:
  nyankowan/can_bus:
    version: "*"
    override_path: "../components/can_bus"
```

### 方法3: 手元のクローンを参照する (ライブラリを修正しながら使う)
このリポジトリをcloneしておき，`main/idf_component.yml` で場所を指定する．
cloneしたファイルを直接参照するので，修正がすぐにプロジェクトに反映される．
```yaml
dependencies:
  nyankowan/damiao:
    version: "*"
    override_path: "/path/to/damiao_for_ESP-IDF/components/damiao"  # main/ からの相対パスも可
  nyankowan/can_bus:
    version: "*"
    override_path: "/path/to/damiao_for_ESP-IDF/components/can_bus"
```

### main以外のコンポーネントから使う場合
使う側のコンポーネントの `CMakeLists.txt` の `REQUIRES` にコンポーネント名を追加する．
```cmake
idf_component_register(SRCS "motor_ctrl.c"
                       INCLUDE_DIRS "."
                       REQUIRES nyankowan__damiao)   # 方法2の場合は damiao
```
| 組み込み方 | コンポーネント名 |
|---|---|
| 方法1 (コンポーネントマネージャ) | `nyankowan__damiao`, `nyankowan__can_bus` (`/` が `__` になる) |
| 方法2 (手動コピー) / 方法3 | `damiao`, `can_bus` (フォルダ名) |

damiao を REQUIRES すれば can_bus も使える (`damiao.h` が `can_bus.h` を include している)．

方法1・3の場合，`idf_component.yml` は `main/` ではなくそのコンポーネントのフォルダに置く．

### 組み込み後の設定
- CANのビットレート: `can_bus_config_t` の `bitrate` で指定する．
- キュー長や受信タスクの優先度: 下記「設定 (menuconfig)」を参照．
- モーターの範囲 (PMAX, VMAX, TMAX): 下記「モーターの範囲」を参照．
- サンプルと同じく1ms単位で制御したい場合は，プロジェクト直下の `sdkconfig.defaults` に `CONFIG_FREERTOS_HZ=1000` を書く (既に `sdkconfig` がある場合は削除してから再ビルド)．

## サンプルをビルドする
```sh
cd examples/basic
idf.py set-target esp32
idf.py build flash monitor
```

### VS Code (ESP-IDF拡張) で開く場合
リポジトリ直下ではなく `examples/basic` フォルダを開く．
初回は `ESP-IDF: Select Current ESP-IDF Version` で使用するIDF (v6.x) を選択し，
clangdを使う場合は `ESP-IDF: Configure clangd` を実行する (`clangd.path` などが各自の環境に合わせて設定される)．
シリアルポートは環境に合わせて `ESP-IDF: Select Port to Use` で変更する．

## 設定 (menuconfig)
`idf.py menuconfig` → `Component config` → `CAN bus dispatcher`

| 項目 | デフォルト | 説明 |
|---|---|---|
| `CONFIG_CAN_BUS_TX_QUEUE_LEN` | 8 | 送信待ちにできるフレーム数 |
| `CONFIG_CAN_BUS_RX_QUEUE_LEN` | 32 | 受信タスクが配送するまで保持できる受信フレーム数 |
| `CONFIG_CAN_BUS_MAX_HANDLERS` | 16 | 登録できるハンドラの数 (モーター1台につき1つ使う) |
| `CONFIG_CAN_BUS_RX_TASK_PRIORITY` | 10 | 受信タスクの優先度．制御タスクと同じか少し高めにする |
| `CONFIG_CAN_BUS_RX_TASK_STACK_SIZE` | 4096 | 受信タスクのスタックサイズ |

受信キューが溢れて破棄されたフレーム数は `can_bus_get_rx_dropped()` や `can_bus_dump_status()` で確認できる．

## モーターの範囲
位置・速度・トルクの範囲はモーターごとに `dm_limits_t` で指定する．
モーター側の設定 (PMAX, VMAX, TMAX) と一致している必要がある．`DM_LIMITS_DEFAULT` は P=12.5, V=45, T=18．
```c
const dm_motor_config_t config = {
    .master_id = 0x11,
    .slave_id  = 0x01,
    .limits    = { .p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f },
};
```

# コードの概要
`can_bus_init` でCANバスを有効化し，`dm_motor_init` でモーターごとのインスタンスを作る．
`dm_motor_enable` でモーターを有効化し，制御ループで指令を送る．
```c:main.c
dm_motor_torque(&s_motor1, 0.5f, 0);
dm_motor_mit(&s_motor2, 0.0f/*position*/, 0.0f/*velocity*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/, 0);
```
`dm_motor_torque` はトルクのみの信号を送る．mitのtorque以外の部分を0にしたときと同じ挙動．\
`dm_motor_mit` はMIT形式で送る．
### MIT 制御式
$$
\tau = K_p(q_d-q) + K_d(\dot q_d-\dot q) + \tau_{ff}
$$

## 受信の仕組み
can_bus の受信タスクが受信フレームを取り出し，登録されたハンドラへ配送する．
```
CANバス → TWAI → [受信タスク] → IDが一致するハンドラ (dm_motor_t など)
                              └→ どれにも一致しない → デフォルトハンドラ
```
- `dm_motor_init()` は内部で `can_bus_register(master_id, ...)` を呼ぶ．
- モーター以外のデバイスは `can_bus_register(id, mask, extd, callback, ctx)` で受信したいIDを登録する．
  `mask` を使えばID範囲をまとめて受け取れる．複数のハンドラに一致した場合はすべてに配送される．
- どこにも登録されていないフレームは `can_bus_set_default_handler()` で受け取れる．
- コールバックは受信タスク上で実行されるので，ブロックする処理や重い処理はしないこと．
  重い処理はキューなどで別タスクに渡す．
- 複数のモーターが同じMASTER_IDを使っている場合も，フィードバックの1バイト目 (SLAVE_IDの下位4bit) で区別される．

## フィードバック
`dm_motor_get_feedback()` で最新のフィードバックを取得する．どのタスクから呼んでもよい．\
`age` で最後に受信してからの経過時間がわかるので，通信断の検知に使える．\
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
## v2.0.0 (複数モーター対応)
TWAIの送受信を can_bus コンポーネントに分離し，モーターIDを直接渡すAPIを廃止した．

| 旧 | 新 |
|---|---|
| `dm_twai_init(tx, rx)` | `can_bus_init(&config)` |
| `dm_twai_deinit()` / `dm_twai_recover()` | `can_bus_deinit()` / `can_bus_recover()` |
| `dm_twai_get_node()` | 廃止 |
| `dm_transmit(can_id, data, wait)` | `can_bus_transmit(&frame, wait)` |
| `dm_enable(SLAVE_ID, wait)` など | `dm_motor_enable(&motor, wait)` など |
| `dm_transmit_mit(SLAVE_ID, ...)` | `dm_motor_mit(&motor, ...)` |
| `dm_transmit_torque(SLAVE_ID, ...)` | `dm_motor_torque(&motor, ...)` |
| `dm_receive(&fb, wait)` | `dm_motor_get_feedback(&motor, &fb, &age)` |
| `dm_pack_mit_cmd(data, ...)` | `dm_pack_mit_cmd(data, &limits, ...)` |
| `dm_dump_twai_status()` | `can_bus_dump_status()` |
| `DM_P_MAX` などのコンパイルオプション | `dm_motor_config_t` の `limits` (モーターごと) |
| menuconfig `DAMIAO motor driver` | menuconfig `CAN bus dispatcher` |

## v1.0.0
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
