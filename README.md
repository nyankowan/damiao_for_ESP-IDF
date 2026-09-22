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

#define SLAVE_ID  0x02

void app_main(void)
{
    ESP_ERROR_CHECK(dm_twai_init(GPIO_NUM_21, GPIO_NUM_22)); // TX, RX
    dm_enable(SLAVE_ID, pdMS_TO_TICKS(10));

    while (1) {
        dm_transmit_mit(SLAVE_ID, 0.0f/*pos*/, 0.0f/*vel*/, 40.0f/*Kp*/, 3.0f/*Kd*/, 0.0f/*torque*/, pdMS_TO_TICKS(1));

        dm_feedback_t fb;
        if (dm_receive(&fb, pdMS_TO_TICKS(10)) == ESP_OK) {
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
├── managed_components/nyankowan__damiao/  ← ダウンロードされたコンポーネント (編集しない．更新時に上書きされる)
└── dependencies.lock                      ← 解決したバージョンの記録
```
`managed_components/` は `.gitignore` に追加してよい．`dependencies.lock` はコミットしておくと全員が同じバージョンでビルドできる．

#### 更新する
```sh
idf.py update-dependencies
```
`dependencies.lock` がある間はビルドしても自動では更新されないため，最新を取り込むときはこのコマンドを実行する．

### 方法2: 手動でコピーする
ネットワークを使わずにビルドしたい場合や，コンポーネントをプロジェクト内で改造したい場合．このリポジトリの `components/damiao/` をフォルダごと，自分のプロジェクトの `components/damiao/` にコピーする．
```
my_robot/
├── CMakeLists.txt
├── components/
│   └── damiao/        ← コピーしたもの
└── main/
```
プロジェクト直下の `components/` はESP-IDFが自動で探すため，`idf_component.yml` の追加は不要．

### 方法3: 手元のクローンを参照する (ライブラリを修正しながら使う)
このリポジトリをcloneしておき，`main/idf_component.yml` で場所を指定する．
cloneしたファイルを直接参照するので，修正がすぐにプロジェクトに反映される．
```yaml
dependencies:
  nyankowan/damiao:
    version: "*"
    override_path: "/path/to/damiao_for_ESP-IDF/components/damiao"  # main/ からの相対パスも可
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
| 方法1 (コンポーネントマネージャ) / 方法3 | `nyankowan__damiao` (`/` が `__` になる) |
| 方法2 (手動コピー) | `damiao` (フォルダ名) |

方法1・3の場合，`idf_component.yml` は `main/` ではなくそのコンポーネントのフォルダに置く．

### 組み込み後の設定
- CANのビットレートやキュー長: 下記「設定 (menuconfig)」を参照．
- モーターの範囲 (`DM_P_MAX`, `DM_T_MAX` など) がDM4310のデフォルトと異なる場合: 下記「設定 (menuconfig)」を参照．
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
