# OdomWheel

n輪オムニホイールの制御、およびオドメトリの計算を行うライブラリです。

## WheelController

オムニホイールの制御を行うクラスです。




## WheelOdometry

オドメトリの計算を行うクラスです。

### 初期化

```cpp
#include <math.h>
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"

// エンコーダーの初期化
Encoder encoder1(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか);
Encoder encoder2(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか);
Encoder encoder3(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか);
Encoder encoder4(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか);

// エンコーダーの位置を設定
WheelConfig config1 = {
    .wheel_radius = 50.0, // mm
    .wheel_x = 100.0, // mm
    .wheel_y = 100.0, // mm
    .wheel_theta = 3 * M_PI / 4 // rad
};
WheelConfig config2 = {
    .wheel_radius = 50.0, // mm
    .wheel_x = -100.0, // mm
    .wheel_y = 100.0, // mm
    .wheel_theta = 5 * M_PI / 4 // rad
};
WheelConfig config3 = {
    .wheel_radius = 50.0, // mm
    .wheel_x = -100.0, // mm
    .wheel_y = -100.0, // mm
    .wheel_theta = 7 * M_PI / 4 // rad
};
WheelConfig config4 = {
    .wheel_radius = 50.0, // mm
    .wheel_x = 100.0, // mm
    .wheel_y = -100.0, // mm
    .wheel_theta = M_PI / 4 // rad
};

// オドメトリの初期化
WheelOdometry<4> odom({config1, config2, config3, config4}, {&encoder1, &encoder2, &encoder3, &encoder4}); // 4輪オムニホイール (n輪の場合はWheelOdometry<n>とする。)
```