#pragma once

#include <mbed.h>
#include "DCMotor/DCMotor.hpp"
#include "Encoder/Encoder.hpp"
#include "PIDController/PIDController.hpp"

class MotorController{
    public:
        MotorController(DCMotor& motor, Encoder& encoder, PIDGain& pid_gain, float max_duty=1.0);
        void setTargetSpeed(float target_rps);
        void stop();

        
        float getSpeed();

    private:
        Ticker ticker;

        DCMotor& motor;
        Encoder& encoder;
        PIDController pid_controller;

        float target_rps = 0;
        float current_rps = 0;
        float last_position = 0;

        float max_duty;

        void loop();
};