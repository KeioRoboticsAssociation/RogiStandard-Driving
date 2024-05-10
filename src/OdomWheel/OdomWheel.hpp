#pragma once
#include "Wheel.hpp"
#include "WheelController.hpp"
#include "Odometry.hpp"

// OdomWheelクラス　測定輪の
template <int N>
class OdomWheel {
    public:
        OdomWheel(const std::array<WheelConfig, N>& wheel_configs, const std::array<MotorController*, N>& motors, const std::array<Encoder*, N>& encoders, std::chrono::microseconds update_interval = std::chrono::microseconds(5000)) 
            : controller(wheel_configs, motors), odometry(wheel_configs, encoders, update_interval)
        {
            controller.setTargetTwist({0.0, 0.0, 0.0});
            odometry.setPose({0.0, 0.0, 0.0});
        }

        WheelController<N> controller;
        Odometry<N> odometry;
};