# MotorController

エンコーダー付きDCモーターを制御するためのライブラリです。

## 使い方


### 初期化

```cpp
#include "DCMotor/DCMotor.hpp"
#include "Encoder/Encoder.hpp"
#include "MotorController/MotorController.hpp"
#include "PIDController/PIDController.hpp"

DCMotor dcmotor(PWMピン, 方向ピン, 方向(0か1));
Encoder encoder(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか);
PIDGain gain = {
    .kp = 1.0, // Pゲイン
    .ki = 0.0, // Iゲイン
    .kd = 0.0, // Dゲイン
    .frequency = 50 // Hz
};
MotorController motor(dcmotor, encoder, gain);
```

### 動かす

```cpp
motor.setTargetSpeed(目標速度); // 目標速度[rps]
```

### 速度を取得する

```cpp
motor.getSpeed(); // 現在の速度[rps]
```




