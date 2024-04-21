# OdomWheel

n輪オムニホイールの制御、およびオドメトリの計算を行うライブラリです。

オドメトリの計算を行うクラスです。

### 初期化

```cpp
#include <mbed.h>
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"
#include "OdomWheel/WheelController.hpp"
#include "DCMotor/DCMotor.hpp"
#include "pins.hpp"
#include "Simulator/MotorSimulator.hpp"

// エンコーダーの初期化
Encoder encoder1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, 0, false);
Encoder encoder2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, 0, false);
Encoder encoder3(InterruptInPins::MEASURING_ENCODER3_A, DigitalInPins::MEASURING_ENCODER3_B, 2048, 0, false);

// DCモーターの初期化
DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);

// モーターコントローラの初期化
MotorController motor1(dc1, encoder1, PIDGain({0.1, 0.01, 0.01, 100}));
MotorController motor2(dc2, encoder2, PIDGain({0.1, 0.01, 0.01, 100}));
MotorController motor3(dc3, encoder3, PIDGain({0.1, 0.01, 0.01, 100}));

// 足回り配置の設定　(ここでは駆動輪を測定輪として使用)
float WHEEL_RAD = 100.0;
float TREAD_RAD = 100.0;
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD, // ホイール半径
        .wheel_x = 0.0, // ホイールのx座標
        .wheel_y = TREAD_RAD, // ホイールのy座標
        .wheel_theta = M_PI //ホイールの向き
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = - M_SQRT3 / 2 * TREAD_RAD,
        .wheel_y = - 0.5 * TREAD_RAD, 
        .wheel_theta = 5 * M_PI / 3
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = + M_SQRT3 / 2 * TREAD_RAD, 
        .wheel_y = - 0.5 * TREAD_RAD,
        .wheel_theta = M_PI / 3
    }
};

// オドメトリ
Odometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});

// 駆動輪
WheelController<3> controller(config, {&motor1, &motor2, &motor3});
```

### オドメトリ
次のようにオドメトリを取得できます。

```cpp
Pose pose = odometry.getPose();
```

ここで`Pose`は、次のような構造体です。

```cpp
typedef struct {
    float x; // 位置(x [mm])
    float y; // 位置(y [mm])
    float theta; // 姿勢(rad)
} Pose;
```

外部からposeを強制的に設定する場合や、ジャイロなどを用いてthetaを書き換える場合、次のようにします。
```cpp
odometry.setPose(pose); // オドメトリを強制的に設定
odometry.setTheta(pose); // 角度を設定
```

### 駆動輪

駆動輪を動かす場合、次のようにします。

```cpp
controller.setTargetTwist({1.0, 1.0, 1.0}); // 目標速度を設定
```

ここで、`Twist`は次のような構造体です。

```cpp
typedef struct {
    float vx; // 速度(x方向 [m/s])
    float vy; // 速度(y方向 [m/s])
    float omega; // 角速度(rad/s)
} Twist;
```



