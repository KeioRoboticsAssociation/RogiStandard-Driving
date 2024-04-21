#include "Encoder.hpp"

Encoder::Encoder(PinName interrupt_in_pin, PinName digital_in_pin, int resolution, bool direction, bool dual) : digital_in(digital_in_pin), interrupt_in(interrupt_in_pin) {
    // カウント数の初期化
    count = 0;
    converted_resolution = resolution * (dual ? 2 : 1) * (direction ? -1 : 1); // 1回転あたりのカウント数を設定

    // 割り込みハンドラの設定
    interrupt_in.rise([this](){interrupt(1);});
    if(dual) {
        interrupt_in.fall([this](){interrupt(0);});
    }
}

// カウント数を取得
int Encoder::getCount() {
    return count;
}

// 角度[rad]を取得
float Encoder::getRadians() {
    return (float)count / converted_resolution * 2 * M_PI;
}

// 角度[deg]を取得
float Encoder::getDegrees() {
    return (float)count / converted_resolution * 360;
}

// 回転数を取得
float Encoder::getRotations() {
    return (float)count / converted_resolution;
}

// 割り込みハンドラ
void Encoder::interrupt(int sgn) {
    if (digital_in.read() == sgn) {
        count++;
    } else {
        count--;
    }
}

// カウント数から角度[rad]に変換
float Encoder::countToRadians(int count)
{
    return (float)count / converted_resolution * 2 * M_PI;
}

// カウント数から角度[deg]に変換
float Encoder::countToDegrees(int count)
{
    return (float)count / converted_resolution * 360;
}

// カウント数から回転数に変換
float Encoder::countToRotations(int count)
{
    return (float)count / converted_resolution;
}

// カウント数を加算 (シミュレーション用)
void Encoder::addCount(int count)
{
    this->count += count;
}

int Encoder::rotationsToCount(float rotations){
    return (int)(rotations * converted_resolution);
}