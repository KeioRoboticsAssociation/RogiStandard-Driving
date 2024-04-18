# DCMotor

DCモーターを未調整(フィードバックなし)で動かせます。
## 使い方

### 初期化

```cpp
#include "DCMotor/DCMotor.hpp"
DCMotor dcmotor(PWMピン, 方向ピン, 方向(0か1))
```

### 動かす

```cpp
dcmotor.setDuty(value); //value: duty比に回転方向の符号をつけたもの -1.0 ~ 1.0
```

