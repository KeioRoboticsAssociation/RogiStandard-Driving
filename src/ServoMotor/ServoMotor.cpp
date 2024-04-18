#include <mbed.h>
#include "ServoMotor.hpp"

ServoMotor::ServoMotor(PinName pin, float pwm_cycle, float pulse_width_min, float pulse_width_max, float range): pwmOut(pin), range(range)
{
    pwmOut.period_us(int(1000*pwm_cycle));
    A = 1000 * (pulse_width_max - pulse_width_min) / range;
    B = 1000 * pulse_width_min;
}

float ServoMotor::convert(float point) const {
    if(point < 0){
        point = 0;
    }else if(point > range){
        point = range;
    }
    return int(A * point + B);
}

void ServoMotor::moveTo(float point){
    pwmOut.pulsewidth_us(convert(point));
}