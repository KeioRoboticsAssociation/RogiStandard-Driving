#pragma once

#include <mbed.h>

class Propeller
{
public:
    Propeller(PinName dirPin, PinName pwmPin);
    PwmOut speed;
    DigitalOut dir;

    void Start(int duty);
};

Propeller::Propeller(PinName dirPin, PinName pwmPin)
    : speed(pwmPin),
      dir(dirPin)
{
    speed.period_ms(20);
    speed.write(0);
    dir.write(1);
}

void Propeller::Start(int duty)
{
    if (duty > 0)
    {
        dir.write(1);
    }
    else
    {
        dir.write(0);
    }
    speed.write(fabs(duty));
}