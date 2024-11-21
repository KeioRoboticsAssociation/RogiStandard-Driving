#include "C620_PWM.hpp"

using namespace C620;

C620_PWM::C620_PWM( PwmOut& _pwm, 
                    bool _direction,
                    InterruptIn& _button, bool _pulluped)
    : pwm(_pwm)
    , direction(_direction)
    , button(_button)
    , is_calibrated_(false)
    , pulluped(_pulluped)
{
    pwm.period_ms(2.5);
    pwm = 0;

    calibration();
    t.attach(callback(this, &C620_PWM::ctrl_callback), 1ms);
}


C620_PWM::~C620_PWM(){
    t.detach();
}

void C620_PWM::calibration(){
    pwm.pulsewidth_us(2000);

    if(this->pulluped){
        button.fall([this]{
            pwm.pulsewidth_us(1000);
            this->button1 = true;
            this->button.disable_irq();
        });
    }
    else{
        button.rise([this]{
            pwm.pulsewidth_us(1000);
            this->button1 = true;
            this->button.disable_irq();
        });
    }
}

void C620_PWM::setVel(float _vel){
    if(_vel > C620::C620_MAX_SPEED){
        vel = C620_MAX_SPEED;
    }else if(_vel < -C620::C620_MAX_SPEED){
        vel = -C620_MAX_SPEED;
    }else{
        vel = _vel;
    }
}

void C620_PWM::ctrl_callback(){
    if(is_calibrated_ & button1 & button2){
        unsigned int pulsewidth = 0;
        if(vel >= 0){
            pulsewidth = 1520 + (1920 - 1520) * vel / C620::C620_MAX_SPEED;
        }
        else{
            pulsewidth = 1480 + (1480 - 1080) * vel / C620::C620_MAX_SPEED;
        }
        pwm.pulsewidth_us(pulsewidth);
    }
    else if(button1){
        if(this->pulluped){
            button.fall([this]{
                pwm.pulsewidth_us(1500); // 双方向モードなら止まる
                this->button2 = true;
                this->button.disable_irq();

                this->is_calibrated_ =true;
            });
        }
        else{
            button.rise([this]{
                pwm.pulsewidth_us(1500);// 双方向モードなら止まる
                this->button2 = true;
                this->button.disable_irq();

                this->is_calibrated_ =true;
            });
        }
    }
    else{
        pwm = 0;
    }
}