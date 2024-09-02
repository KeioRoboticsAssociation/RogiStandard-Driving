#pragma once

#include <mbed.h>

// 電磁弁を制御するクラス
class AirCylinder
{
public:
    // コンストラクタで制御ピンを設定
    AirCylinder(PinName pin1);

    // 電磁弁をオンにする
    void on();
    // 電磁弁をオフにする
    void off();

    // 電磁弁をオンにして指定した時間待機する
    void Start(int value);

private:
    DigitalOut cylinder1; // 電磁弁制御用のデジタル出力
    int value;
};

// コンストラクタで制御ピンを設定
AirCylinder::AirCylinder(PinName pin1)
    : cylinder1(pin1)
{
}

void AirCylinder::on()
{
    cylinder1.write(1);
}

void AirCylinder::off()
{
    cylinder1.write(0);
}

void AirCylinder::Start(int value)
{
    wait_us(value);
}