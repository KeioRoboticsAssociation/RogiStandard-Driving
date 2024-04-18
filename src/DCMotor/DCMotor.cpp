#include "DCMotor.hpp"

DCMotor::DCMotor(PinName pwm_pin, PinName dir_pin, bool direction, float pwm_freq) : pwm(pwm_pin), dir(dir_pin) {
    pwm.period(1.0 / pwm_freq); // PWM周期を設定
    setDuty(0.0); // デューティ比を初期化
    this->direction = direction; // 回転方向を設定
}

// duty比を設定
void DCMotor::setDuty(float duty) {
    last_duty = duty;

    if (direction) {
        duty = -duty; // 逆回転の場合はデューティ比を反転
    }

    pwm.write(fabs(duty)); // デューティ比を設定
    dir.write(duty > 0 ? 0 : 1); // 回転方向を設定
}

// duty比を取得
float DCMotor::getDuty() {
    return last_duty;
}