#pragma once
#include <mbed.h>

class ServoMotor{
    public:
        ServoMotor(PinName pin, float pwm_cycle=20, float pulse_width_min=0.5, float pulse_width_max=2.4, float range=180);

        PwmOut pwmOut;
        void moveTo(float point);
        float convert(float point) const;
        
        float A = 0;
        float B = 0;
    private:
        //y:pwm幅，x:角度として，y = Ax+B
        
        float range;
};