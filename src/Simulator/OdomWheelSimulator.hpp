// WORK IN PROGRESS

#pragma once
#include "OdomWheel/Wheel.hpp"
#include "OdomWheel/WheelController.hpp"
#include "MotorSimulator.hpp"

template <int wheel_num, typename Odometry> 
class OdomWheelSimulator {
    public:
        OdomWheelSimulator(WheelController<wheel_num>& wheel_controller, Odometry& odometry, Pose initial_pose={0.0,0.0,0.0})
         : wheel_controller(wheel_controller), odometry(odometry)
        {
            odometry.setPose(initial_pose);
            for (int i=0;i<wheel_num;i++) {
                motor_simulators[i] = wheel_controller.motors[i];
            }
        }

        void loop() {
            pose = odometry.getPose();
            std::array<float, wheel_num> speeds;
            for (int i=0;i<wheel_num;i++){
                motor_simulators[i].loop();
                speeds[i] = motor_simulators[i].getSpeed();
            }   
        }

    private:
        WheelController<wheel_num>& wheel_controller;
        Odometry& odometry;
        Pose pose;

        std::array<MotorSimulator, wheel_num> motor_simulators;
};