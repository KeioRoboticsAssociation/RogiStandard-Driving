// #pragma once

// #include "OdomWheel/Odometry.hpp"
// #include "OdomWheel/WheelController.hpp"

// #include "PIDController/PIDController.hpp"


// class DriveBase {
//     public:
//         enum class DriveState {
//             IDLE, // 待機
//             TWIST_TRACKING, // 速度追従
//             POINT_TRACKING, // 点追従
//         };

//         DriveBase(ThreeWheelController& wheel_controller, ThreeWheelOdometry& odometry);

//         void setTargetPose(const Pose& target_pose);
//         void setTargetTwist(const Twist& target_twist);

//         void twistTrackingLoop();
//         void pointTrackingLoop();

//         void loop();



//     private:
//         ThreeWheelController& wheel_controller;
//         ThreeWheelOdometry& odometry;

//         Ticker ticker;

//         DriveState state = DriveState::IDLE; // 現在の状態

//         Pose target_pose;
//         Twist target_twist;

//         PIDController pid_controller_trans{
//             PIDGain{
//                 .kp = 0.1, // P
//                 .ki = 0.0, // I
//                 .kd = 0.0, // D
//             }
//         };
//         PIDController pid_controller_rot{
//             PIDGain{
//                 .kp = 0.1, // P
//                 .ki = 0.0, // I
//                 .kd = 0.0, // D
//             }
//         };
//         float dt;
// };




