#include <mbed.h>
#include "WTT12L.hpp"

WTT12L::WTT12L(PinName pinA, PinName pinB) : rangefinder1(pinA), rangefinder2(pinB) {
    // 初期化
}

int WTT12L::getOutput1() {
    // 近いと1、遠いと0を出力
    int output = rangefinder1.read();
    return output;
}

int WTT12L::getOutput2() {
    // 近いと1、遠いと0を出力
    int output = rangefinder2.read();
    return output;
}

