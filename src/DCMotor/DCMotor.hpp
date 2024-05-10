#pragma once

#include <mbed.h>

class DCMotor {
    public:
        DCMotor(PinName pwm_pin, PinName dir_pin, bool direction=0, float pwm_freq=16000 /* Hz */);
        void setDuty(float duty);
        float getDuty();

        PwmOut pwm;
        DigitalOut dir;
        
    private:

        bool direction;
        float last_duty;
};