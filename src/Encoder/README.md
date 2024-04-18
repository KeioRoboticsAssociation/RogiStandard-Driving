# Encoder

インクリメント型ロータリエンコーダーを読むためのライブラリです。

## 使い方

### 初期化
```cpp
Encoder encoder(Aピン(割り込み), Bピン(デジタル), 分解能, 方向(0か1), 立ち下がりも読むか)
```

### 値の読み取り
```cpp
encoder.getCount(); // カウント値
encoder.getRadians(); // 角度[rad]
encoder.getDegrees(); // 角度[deg]
encoder.getRotations(); // 回転数
```