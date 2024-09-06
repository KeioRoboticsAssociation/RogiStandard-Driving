// includeする
#include <mbed.h>
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"
#include "OdomWheel/WheelController.hpp"
#include "DCMotor/DCMotor.hpp"
#include "pins.hpp"
#include "Simulator/MotorSimulator.hpp"
#include "MotorController/MotorController.hpp"
#include "PIDController/PIDController.hpp"
#include "WTT12L/WTT12L.hpp"
#include "WTT12L/WTT12LBack.hpp"
#include "BNO055/BNO055.hpp"
#include "AirCylinder/AirCylinder.hpp"
#include "Propeller/Propeller.hpp"
#include "parameters.hpp"
#include "settings.hpp"

#if !PERFORMANCE_ROBOT
// 測定輪encoderの設定
Encoder encoder_1(InterruptInPins::ENCODER_FL_A, DigitalInPins::ENCODER_FL_B, 2048, 1, false);
Encoder encoder_2(InterruptInPins::ENCODER_FR_A, DigitalInPins::ENCODER_FR_B, 2048, 1, false);
Encoder encoder_3(InterruptInPins::ENCODER_BL_A, DigitalInPins::ENCODER_BL_B, 2048, 1, false);
#endif

#if PERFORMANCE_ROBOT
// 測定輪encoderの設定
Encoder encoder_1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, 1, false);
Encoder encoder_2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, 1, false);

// encoderの設定
Encoder encoder_FL(InterruptInPins::ENCODER_FL_A, DigitalInPins::ENCODER_FL_B, 2048, 1, false);
Encoder encoder_FR(InterruptInPins::ENCODER_FR_A, DigitalInPins::ENCODER_FR_B, 2048, 1, false);
Encoder encoder_BL(InterruptInPins::ENCODER_BL_A, DigitalInPins::ENCODER_BL_B, 2048, 1, false);
Encoder encoder_BR(InterruptInPins::ENCODER_BR_A, DigitalInPins::ENCODER_BR_B, 2048, 1, false);
#endif

// DCモータの設定
#if !PERFORMANCE_ROBOT
DCMotor dc_FL(PwmOutPins::MOTOR_FL_PWM, DigitalOutPins::MOTOR_FL_DIR, 0);
DCMotor dc_FR(PwmOutPins::MOTOR_FR_PWM, DigitalOutPins::MOTOR_FR_DIR, 0);
DCMotor dc_BL(PwmOutPins::MOTOR_BL_PWM, DigitalOutPins::MOTOR_BL_DIR, 0);
#endif

#if PERFORMANCE_ROBOT
DCMotor dc_FL(PwmOutPins::MOTOR_FL_PWM, DigitalOutPins::MOTOR_FL_DIR, 0);
DCMotor dc_FR(PwmOutPins::MOTOR_FR_PWM, DigitalOutPins::MOTOR_FR_DIR, 0);
DCMotor dc_BL(PwmOutPins::MOTOR_BL_PWM, DigitalOutPins::MOTOR_BL_DIR, 0);
DCMotor dc_BR(PwmOutPins::MOTOR_BR_PWM, DigitalOutPins::MOTOR_BR_DIR, 0);
#endif

// モーターコントローラの設定
#if !PERFORMANCE_ROBOT
MotorController motor_FL(dc_FL, encoder_1, {0.16, 0.01, 0.01, 20});
MotorController motor_FR(dc_FR, encoder_2, {0.16, 0.01, 0.01, 20});
MotorController motor_BL(dc_BL, encoder_3, {0.16, 0.01, 0.01, 20});
#endif

#if PERFORMANCE_ROBOT
MotorController motor_FL(dc_FL, encoder_FL, {0.16, 0.01, 0.01, 20});
MotorController motor_FR(dc_FR, encoder_FR, {0.16, 0.01, 0.01, 20});
MotorController motor_BL(dc_BL, encoder_BL, {0.16, 0.01, 0.01, 20});
MotorController motor_BR(dc_BR, encoder_BR, {0.16, 0.01, 0.01, 20});
#endif

#if USE_PROPELLER
Propeller propeller(DigitalOutPins::PROPELLER_MOTOR_DIR, PwmOutPins::PROPELLER_MOTOR_PWM);
#endif

#if USE_AIRCYLINDER
AirCylinder aircylinder(DigitalOutPins::CYLINDER1);
#endif

#if USE_LASER_WTT12L
WTT12L backlaser(DigitalInPins::RIGHT_WTT12L_1, DigitalInPins::RIGHT_WTT12L_2);
WTT12LBack rightlaser(DigitalInPins::BACK_WTT12L);
#endif

#if USE_GYROSENSOR_BNO055
BNO055 gyrosensor(UartPins::CONSOLE_TX, UartPins::CONSOLE_RX);
#endif

// オムニホイールの配置を設定
#if !PERFORMANCE_ROBOT
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RADIUS, // 車輪の半径
        .wheel_x = 0.0,               // 車輪のx座標
        .wheel_y = TRED_RADIUS,       // 車輪のy座標
        .wheel_theta = M_PI           // 車輪の角度
    },
    WheelConfig{
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = -M_SQRT3 / 2 * TRED_RADIUS,
        .wheel_y = -0.5 * TRED_RADIUS,
        .wheel_theta = 5 * M_PI / 3},
    WheelConfig{
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = +M_SQRT3 / 2 * TRED_RADIUS,
        .wheel_y = -0.5 * TRED_RADIUS,
        .wheel_theta = M_PI / 3}};
#endif

#if PERFORMANCE_ROBOT
#if USE_MEASURING_WHEEL
std::array<WheelConfig, 2> measuring_config = {
    WheelConfig{
        // x方向
        .wheel_radius = WHEEL_RADIUS, // 車輪の半径
        .wheel_x = +156.6,            // 車輪のx座標
        .wheel_y = +2.344,            // 車輪のy座標
        .wheel_theta = 0.005 * M_PI   // 車輪の角度
    },
    WheelConfig{// y方向
                .wheel_radius = WHEEL_RADIUS,
                .wheel_x = -198.1,
                .wheel_y = +36.00,
                .wheel_theta = 0.943 * M_PI}};
#endif
std::array<WheelConfig, 4> config = {
    WheelConfig{
        // FL
        .wheel_radius = WHEEL_RADIUS,        // 車輪の半径
        .wheel_x = +SQRT2 / 2 * TRED_RADIUS, // 車輪のx座標
        .wheel_y = +SQRT2 / 2 * TRED_RADIUS, // 車輪のy座標
        .wheel_theta = M_PI / 4 * 7          // 車輪の角度
    },
    WheelConfig{// FR
                .wheel_radius = WHEEL_RADIUS,
                .wheel_x = +SQRT2 / 2 * TRED_RADIUS,
                .wheel_y = -SQRT2 / 2 * TRED_RADIUS,
                .wheel_theta = M_PI / 4 * 5},
    WheelConfig{// BL
                .wheel_radius = WHEEL_RADIUS,
                .wheel_x = -SQRT2 / 2 * TRED_RADIUS,
                .wheel_y = +SQRT2 / 2 * TRED_RADIUS,
                .wheel_theta = M_PI / 4},
    WheelConfig{// BR
                .wheel_radius = WHEEL_RADIUS,
                .wheel_x = -SQRT2 / 2 * TRED_RADIUS,
                .wheel_y = -SQRT2 / 2 * TRED_RADIUS,
                .wheel_theta = M_PI / 4 * 3}};
#endif

// オドメトリとホイールコントローラの設定
#if !PERFORMANCE_ROBOT
Odometry<3> odometry(config, {&encoder_1, &encoder_2, &encoder_3});
WheelController<3> controller(config, {&motor_FL, &motor_FR, &motor_BL});
#endif
#if PERFORMANCE_ROBOT
Odometry<4> odometry(config, {&encoder_FL, &encoder_FR, &encoder_BL, &encoder_BR});
WheelController<4> controller(config, {&motor_FL, &motor_FR, &motor_BL, &motor_BR});
DigitalIn start_sw(DigitalInPins::START_SWITCH);
#endif
PIDController robot_pose_pid_x(0.4, 0.0, 0.0, 20);
PIDController robot_pose_pid_y(0.4, 0.0, 0.0, 20);
PIDController robot_pose_pid_theta(0.4, 0.0, 0.0, 20);

PIDController robot_velocity_pid_x(0.4, 0.0, 0.0, 20);
PIDController robot_velocity_pid_y(0.4, 0.0, 0.0, 20);
PIDController robot_velocity_pid_theta(0.4, 0.0, 0.0, 20);
Timer timer;
Ticker ticker;

float vx, vy, vtheta;
Pose current_pose;

void stop()
{
    motor_FL.stop();
    motor_FR.stop();
    motor_BL.stop();
    motor_BR.stop();
}

void robot_twist(float target_x, float target_y, float target_theta, float x, float y, float theta)
{
    // target_x単位はmm
    // kp1, kd,ki 0.01くらい
    vx = robot_velocity_pid_x.calculate(robot_pose_pid_x.calculate(target_x - x));
    vy = robot_velocity_pid_y.calculate(robot_pose_pid_y.calculate(target_y - y));
    vtheta = robot_velocity_pid_theta.calculate(robot_pose_pid_theta.calculate(target_theta - theta));
    controller.setTargetTwist({vx, vy, vtheta});
    // printf("vx: %d, vy: %d, vtheta: %d\n", (int)vx, (int)vy, (int)vtheta)
    ;
}

void robot_twist_up(float max_v, float accx, float accy, float last_x, float last_y, float target_x, float target_y, float target_theta, float x, float y, float theta)
{
    // target_x単位はmm
    // kp1, kd,ki 0.01くらい

    if (((y - last_y) <= (target_y - y)))
    {
        printf("1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx * (current_time);
        vy = accy * (current_time);
        controller.setTargetTwist({vx, vy, 0});
        printf("last_x: %d, last_y: %d, x: %d, y: %d, target_x: %d, target_y: %d\n", (int)last_x, (int)last_y, (int)x, (int)y, (int)target_x, (int)target_y);
    }
    else if (((0 < (target_y - y)) && ((target_y - y) < (y - last_y))))
    {
        printf("2nd\n");

        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);

        controller.setTargetTwist({robot_pose_pid_x.calculate(target_x - x), robot_pose_pid_y.calculate(target_y - y), robot_pose_pid_theta.calculate(target_theta - theta)});
    }
    else
    {
        printf("3rd\n");
        printf("last_x: %d, x: %d, last_y: %d, y: %d\n", (int)last_x, (int)x, (int)last_y, (int)y);
        stop();
    }
}

float distanceError = 0.0;
float thetaError = 0.0;
float threshold = 200.0;
float targetx = 0;
float targety = 1700;
float targetAngle;

#if USE_LASER_WTT12L
// bool adjustByRightlaser()
// {
//     while (1)
//     {
//         if (rightlaser.getOutput1() == 1) // ターゲット距離より近い
//         {
//             // 現在の距離が目標より小さい場合、左に移動
//             controller.setTargetTwist({0, 0.5, 0});
//             return false;
//         }
//         else if (rightlaser.getOutput2() == 0) // ターゲット距離より遠い
//         {
//             // 現在の距離が目標より大きい場合、右に移動
//             controller.setTargetTwist({0, -0.5, 0});
//             return false;
//         }
//         else
//         {
//             current_pose.x = 0;
//             stop();
//             printf("Position adjusted: Distance to wall = %.2f mm\n", 0.075 - current_pose.x);
//             return true;
//             break;
//         }
//         wait_us(100); // 100ms
//     }
// }
// bool adjustByBacklaser()
// {
//     while (1)
//     {
//         if (backlaser.getOutput() == 0) // ターゲット距離より遠い
//         {
//             // 現在の距離が目標より大きい場合、右に移動
//             controller.setTargetTwist({0, 0.5, 0});
//             return false;
//         }
//         else
//         {
//             current_pose.x = 0;
//             stop();
//             printf("Position adjusted: Distance to wall = %.2f mm\n", 0.075 - current_pose.x);
//             return true;
//             break;
//         }
//         wait_us(100); // 100ms
//     }
// }
#endif
bool forward_1700()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(pow(1700 - current_pose.x, 2.0) + pow(current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1700, 0, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
// bool scanRight()
// {
// #if USE_LASER_WTT12L
//     if (adjustByRightlaser() == false)
//     {
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// #endif
// }
bool stop_30()
{
    float current_time = timer.read();
    if (current_time < 30)
    {
        stop();
#if USE_PROPELLER
        propeller.Start(0.5);
#endif
        return false;
    }
    else
    {
#if USE_PROPELLER
        propeller.Start(0);
#endif
        return true;
    }
}
bool backward_500()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(1200 - pow(current_pose.x, 2.0) + pow(current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1200, 0, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
bool left_650()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(pow(1200 - current_pose.x, 2.0) + pow(650 - current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1200, 650, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
bool forward_500()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(pow(1700 - current_pose.x, 2.0) + pow(650 - current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1700, 650, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
bool backward_750()
{
// is_moving = true;
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(pow(1700 - 750 - current_pose.x, 2.0) + pow(650 - current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1700 - 750, 650, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
// bool scanBack()
// {
// #if USE_LASER_WTT12L
//     if (adjustByBacklaser() == false)
//     {
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// #endif
// }
bool rotate_90()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(-M_PI / 2);
#endif
    thetaError = -M_PI / 2 - current_pose.theta;
    if (thetaError > -M_PI / 16)
    {
        robot_twist(1700 - 750, 650, -M_PI / 2, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
#if USE_GYROSENSOR_BNO055
        gyrosensor.reset();
#endif
        return true;
    }
}
bool left_950()
{
#if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
#endif
    distanceError = sqrt(pow(1700 - 750 - current_pose.x, 2.0) + pow(650 + 950 - current_pose.y, 2.0));
    if (distanceError > threshold)
    {
        robot_twist(1700 - 750, 650 + 950, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        return false;
    }
    else
    {
        return true;
    }
}
bool release()
{
    float current_time = timer.read();
    if (current_time < 60)
    {
        stop();
#if USE_AIRCYLINDER
        aircylinder.on();
#endif
        return false;
    }
    else
    {
#if USE_AIRCYLINDER
        aircylinder.off();
#endif
        return true;
    }
}

enum Mode
{
    INIT,
    FORWARD_1700,
    SCAN_RIGHT,
    STOP_30_1,
    BACKWARD_500,
    LEFT_650,
    FORWARD_500,
    SCAN_RIGHT_2,
    STOP_30_2,
    BACKWARD_750,
    SCAN_BACK,
    ROTATE,
    LEFT_950,
    WAIT, // 待つ？どれくらいで到着するかによる
    RELEASE,
    STOP
};

Mode currentMode = Mode::FORWARD_1700;
Mode previous_mode = Mode::FORWARD_1700;

void updateMode(Mode new_mode)
{
    if (new_mode != currentMode)
    {
        // モードが切り替わったときのみタイマーをリセット
        timer.reset();
        timer.start();

        // モードが切り替わった後に更新
        previous_mode = currentMode;
        currentMode = new_mode;
    }
}

void update()
{
    // アクションの切り替え
    switch (currentMode)
    {
    case Mode::INIT:
        currentMode = Mode::FORWARD_1700;
        break;
    case Mode::FORWARD_1700:
        if (forward_1700())
        {
            updateMode(Mode::SCAN_RIGHT);
            currentMode = Mode::SCAN_RIGHT;
        }
        break;
    case Mode::SCAN_RIGHT:
        // if (scanRight())
        // {
        //     updateMode(Mode::STOP_30_1);
        //     currentMode = Mode::STOP_30_1;
        // }
        break;
    case Mode::STOP_30_1:
        if (stop_30())
        {
            updateMode(Mode::BACKWARD_500);
            currentMode = Mode::BACKWARD_500;
        }
        break;
    case Mode::BACKWARD_500:
        if (backward_500())
        {
            updateMode(Mode::LEFT_650);
            currentMode = Mode::LEFT_650;
        }
        break;
    case Mode::LEFT_650:
        if (left_650())
        {
            updateMode(Mode::FORWARD_500);
            currentMode = Mode::FORWARD_500;
        }
        break;
    case Mode::FORWARD_500:
        if (forward_500())
        {
            updateMode(Mode::SCAN_RIGHT_2);
            currentMode = Mode::SCAN_RIGHT_2;
        }
        break;
    case Mode::SCAN_RIGHT_2:
        //
        break;
    case Mode::STOP_30_2:
        if (stop_30())
        {
            updateMode(Mode::BACKWARD_750);
            currentMode = Mode::BACKWARD_750;
        }
        break;
    case Mode::BACKWARD_750:
        if (backward_750())
        {
            updateMode(Mode::SCAN_BACK);
            currentMode = Mode::SCAN_BACK;
        }
        break;
    case Mode::SCAN_BACK:
        // if (scanBack())
        // {
        //     updateMode(Mode::ROTATE);
        //     currentMode = Mode::ROTATE;
        // }
        break;
    case Mode::ROTATE:
        if (rotate_90())
        {
            updateMode(Mode::LEFT_950);
            currentMode = Mode::LEFT_950;
        }
        break;
    case Mode::LEFT_950:
        if (left_950())
        {
            updateMode(Mode::RELEASE);
            currentMode = Mode::RELEASE;
        }
        break;
    case Mode::RELEASE:
        if (release())
        {
            updateMode(Mode::STOP);
            currentMode = Mode::STOP;
        }
    case Mode::STOP:
        // 何もしない
        break;
    }
}

bool is_moivng = false;

int main()
{
    printf("System ready. Waiting for button press...\n");

    while (start_sw == 1)
    {
        ThisThread::sleep_for(100ms);
    }
    printf("Button pressed! Starting operation...\n");

    // printf("Encoder fl: %d, fr: %d, bl: %d, br: %d, measur1: %d, measur2: %d\n", (int)encoder_FL.getCount(), (int)encoder_FR.getCount(), (int)encoder_BL.getCount(), (int)encoder_BR.getCount(), (int)encoder_1.getCount(), (int)encoder_2.getCount());

    // printf("bno output: %d\n", (int)gyrosensor.getRadians() * 1000);

    // current_pose = odometry.getPose();
    // printf("pos: %d, %d, %d\n", (int)(current_pose.x*100), (int)(current_pose.y*100), (int)(current_pose.theta*100));
    // printf("difference_x = %d, difference_y = %d, difference_theta = %d\n", (int)current_pose.x, 1700 - (int)current_pose.y, (int)gyrosensor.getRadians() * 1000);
    // controller.setTargetTwist({0.0, 0.0, 1.0});

    // vx = robot_velocity_pid_x.calculate(robot_pose_pid_x.calculate(1700 - current_pose.x));
    // vy = robot_velocity_pid_y.calculate(robot_pose_pid_y.calculate(0 - current_pose.y));
    // vtheta = robot_velocity_pid_theta.calculate(robot_pose_pid_theta.calculate(0 - current_pose.theta));
    // controller.setTargetTwist({vx, vy, vtheta});
    // controller.setTargetTwist({40, 0, 0});

    // printf("pidx: %d, pidy: %d, pidtheta: %d\n", (int)robot_pose_pid_x.calculate(1700 - current_pose.x), (int)robot_pose_pid_y.calculate(0 - current_pose.y), (int)robot_pose_pid_theta.calculate(0 - current_pose.theta));
    // printf("vx: %d, vy: %d, vtheta: %d\n", (int)vx, (int)vy, (int)vtheta);
    // printf("%d\n", (int)current_pose.theta*1000);

    // motor_FL.setTargetSpeed(3.0);
    // motor_FR.setTargetSpeed(3.0);
    // motor_BL.setTargetSpeed(3.0);
    // motor_BR.setTargetSpeed(3.0);
    aircylinder.on();
    controller.setTargetTwist({40, 0, 0});
    printf("40.0, 0, 0\n");
    int i;  
    while (backlaser.getOutput1())
    {
        ThisThread::sleep_for(10ms);
        printf("backlaser1 on\n");
        i++;
        printf("%d ", i);
    }
    // mawasu

    // おそらく時間が３０秒以下の間はプロペラを回す
    // ちょっと長そうなので20秒にしてみた
    // 30秒を超えたところでプロペラを止める
    timer.reset();
    // while(1){
    //     if (timer.read() < 20)
    //     {
    //         stop();
    //         #if USE_PROPELLER
    //         propeller.Start(0.5);
    //         printf("propeller moving.\n");
    //         #endif
    //     }
    //     else
    //     {
    //         #if USE_PROPELLER
    //         propeller.Start(0);
    //         printf("propeller stopped.\n");
    //         #endif
    //         break;
    //     }   
    // }
//     if (current_time < 30)
//     {
//         stop();
// #if USE_PROPELLER
//         propeller.Start(0.5);
// #endif
//         return false;
//     }
//     else
//     {
// #if USE_PROPELLER
//         propeller.Start(0);
// #endif
//         return true;
//     }
    // timer.reset();
    // timer.start();
    // while(1){
    //     if (timer.read() < 20)
    //     {
    //         stop();
    //         #if USE_PROPELLER
    //         propeller.Start(0.5);
    //         printf("propeller moving.");
    //         #endif
    //     }
    //     else
    //     {
    //         #if USE_PROPELLER
    //         propeller.Start(0);
    //         printf("propeller stopped.");
    //         #endif
    //         break;
    //     }   
    // }
    printf("stop_30\n");

    controller.setTargetTwist({-40.0, 0, 0});
    printf("-40.0, 0, 0\n");
    while (!backlaser.getOutput2())
    {
        ThisThread::sleep_for(10ms);
        printf("backlaser2 on\n");
    }

    controller.setTargetTwist({0, 40.0, 0});
    ThisThread::sleep_for(std::chrono::milliseconds(3000));
    printf("0, 40.0, 0\n");

    controller.setTargetTwist({40, 0, 0});
    ThisThread::sleep_for(std::chrono::milliseconds(3000));
    printf("40, 0, 0\n");

    // mawasu

    controller.setTargetTwist({-40, 0, 0});
    ThisThread::sleep_for(std::chrono::milliseconds(3000));
    printf("-40, 0, 0\n");

    controller.setTargetTwist({0, 40, 0});
    while (rightlaser.getOutput())
    {
        ThisThread::sleep_for(10ms);
        printf("rightlaser on\n");
    }

    controller.setTargetTwist({0, 0, -1.0});
    printf("0, 0, -1.0\n");
    ThisThread::sleep_for(3100ms);
    release();

    // while(1){
    //     printf("backlaser1: %d, backlaser2: %d, rightlaser: %d\n", backlaser.getOutput1(), backlaser.getOutput2(), rightlaser.getOutput());
    //     ThisThread::sleep_for(100ms);
    // }

#if TEST
    robot_twist_up(0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
#endif

#if !TEST
    // update();
#endif
}
