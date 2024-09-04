#include <mbed.h>
#include "WTT12LBack.hpp"

WTT12LBack::WTT12LBack(PinName pinA) : rangefinder(pinA) {
    // 初期化
}

int WTT12LBack::getOutput() {
    // 近いと1、遠いと0を出力
    int output = rangefinder.read();
    return output;
}
