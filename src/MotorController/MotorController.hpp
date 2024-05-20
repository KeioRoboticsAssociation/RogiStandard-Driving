#pragma once

#include <mbed.h>
#include "DCMotor/DCMotor.hpp"
#include "Encoder/Encoder.hpp"
#include "PIDController/PIDController.hpp"

class MotorController{
    public:
        MotorController(DCMotor& motor, Encoder& encoder, PIDGain pid_gain=PIDGain{0.13, 0.0, 0.0, 20}, float max_duty=1.0);
        void setTargetSpeed(float target_rps);
        void stop();

        
        float getSpeed();
        float getTargetSpeed();

        DCMotor& motor;
        Encoder& encoder;
        PIDController pid_controller;

    private:
        Ticker ticker;
        

        float target_rps = 0;
        float current_rps = 0;
        float last_position = 0;
        float last_duty = 0;

        float max_duty;

        bool moving = false;
        

        void loop();
};