// #include "DriveBase.hpp"
// #include "math.h"

// DriveBase::DriveBase(ThreeWheelController& wheel_controller, ThreeWheelOdometry& odometry)
//     : wheel_controller(wheel_controller), odometry(odometry)
// {
//     dt = odometry.getUpdateInterval().count() / 1000000.0; // 更新周期を計算
//     pid_controller_trans.setFrequeny(1.0 / dt); // PID制御器の更新周期を設定
//     pid_controller_rot.setFrequeny(1.0 / dt); // PID制御器の更新周期を設定
//     ticker.attach(callback(this, &DriveBase::loop), odometry.getUpdateInterval());
// }

// void DriveBase::setTargetPose(const Pose& target_pose)
// {
//     this->target_pose = target_pose;
//     state = DriveState::POINT_TRACKING;
// }

// void DriveBase::setTargetTwist(const Twist& target_twist)
// {
//     this->target_twist = target_twist;
//     state = DriveState::TWIST_TRACKING;
// }


// #define TWIST_TRACKING_WITH_POINT_TRACKING (1)

// // 速度制御のループ
// void DriveBase::twistTrackingLoop()
// {
//     #if TWIST_TRACKING_WITH_POINT_TRACKING
    
//     // 現在の位置を取得
//     Pose pose = odometry.getPose();

//     // 目標位置を計算する
//     Pose new_target_pose; 
//     new_target_pose.x = pose.x + target_twist.vx * dt;
//     new_target_pose.y = pose.y + target_twist.vy * dt;
//     new_target_pose.theta = pose.theta + target_twist.omega * dt;

//     // 目標位置を設定
//     setTargetPose(new_target_pose); 

//     // 位置制御と同じ処理を行う
//     pointTrackingLoop();

//     #else

    


//     #endif
// }

// // 位置制御のループ
// void DriveBase::pointTrackingLoop()
// {
//     // 目標位置と現在位置の差を計算
//     Pose current_pose = odometry.getPose();
//     float dx = target_pose.x - current_pose.x;
//     float dy = target_pose.y - current_pose.y;
//     float dtheta = target_pose.theta - current_pose.theta;

//     // 目標位置への距離と角度を計算
//     float distance = sqrt(dx * dx + dy * dy);
//     float angle = atan2(dy, dx);

//     // PID制御器で速度と角速度を計算
//     float speed = pid_controller_trans.calculate(distance);
//     float omega = pid_controller_rot.calculate(dtheta);

//     // x軸方向の速度とy軸方向の速度を計算
//     float vx = speed * cos(angle);
//     float vy = speed * sin(angle);

//     // ロボットの速度を計算
//     Twist global_twist = Twist{vx, vy, omega};

//     // ロボットの速度をローカル座標系に変換
//     Twist local_twist = global_twist.rotate(-current_pose.theta);

//     // ロボットの速度を設定
//     wheel_controller.setTargetTwist(local_twist);
// }

// // ループ関数
// void DriveBase::loop()
// {
//     switch (state) {
//         case DriveState::IDLE:
//             break;
//         case DriveState::TWIST_TRACKING:
//             twistTrackingLoop();
//             break;
//         case DriveState::POINT_TRACKING:
//             pointTrackingLoop();
//             break;
//     }
// }