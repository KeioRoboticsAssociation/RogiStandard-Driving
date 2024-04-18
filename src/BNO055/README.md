# BNO055

9軸センサBNO055を読むためのライブラリです

## 使い方

### 初期化

```cpp
#include "BNO055/BNO055.hpp"

BNO055 bno055(TXピン, RXピン);
```

### 値の読み取り

```cpp
bno055.getRadians(); // 角度[rad]
bno055.getDegrees(); // 角度[deg]
```

### 値の設定

```cpp
bno055.setRadians(value); // 角度をvalue[rad]に設定
bno055.reset(); // 角度を0に設定
```
