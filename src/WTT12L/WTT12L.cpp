#include <mbed.h>
#include "WTT12L.hpp"

WTT12L::WTT12L(PinName pinA) : rangefinder(pinA) {
    // 初期化
}

bool WTT12L::whetherclose(float target_threshold) {
    if (rangefinder > target_threshold) {
        printf("close\n");
        wait_us(500000);

        return true;
    } else {
        printf("far\n");
        wait_us(500000);

        return false;
    }
}

