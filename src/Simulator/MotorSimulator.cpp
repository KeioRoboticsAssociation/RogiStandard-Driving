#include "MotorSimulator.hpp"

MotorSimulator::MotorSimulator(DCMotor& motor, Encoder* encoder, float gain, float timeConstant, float deadTime, float interval) 
: motor(motor), encoder(encoder), queue(int(deadTime / interval))
{
    this->interval = interval;
    this->constantA = timeConstant / (timeConstant + interval);
    this->constantB = gain * this->constantA;
}

void MotorSimulator::loop() {
    float input = this->motor.getDuty();
    input = this->queue.pop(input); // むだ時間を考慮

    output = this->constantA * last_output + this->constantB * input; // 伝達関数
    last_output = output;
    
    if (encoder != nullptr) {
        encoder->addCount(encoder->rotationsToCount(output * interval));
    }
}

float MotorSimulator::getSpeed(){
    return output;
}


MotorSimulator::~MotorSimulator() {
    
}
