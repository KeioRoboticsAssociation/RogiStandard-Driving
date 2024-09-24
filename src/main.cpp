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

float distanceError = 0.0;
float thetaError = 0.0;
float threshold = 200.0;
float targetx = 0;
float targety = 1700;
float targetAngle;

bool forward_1400()
{
    float current_time = timer.read();
    if (current_time <= 10)
    {
        controller.setTargetTwist({100, 0, 0});
        return false;
    }
    else
    {
        stop();
        return true;
    }
}
bool stop_20()
{
    if (backlaser.getOutput1())
    {
        controller.setTargetTwist({90, 0,0});
#if USE_PROPELLER
        propeller.Start(-0.5);
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
    if (!backlaser.getOutput2())
    {
        controller.setTargetTwist({-70.0, 0, 0});
        printf("-40.0, 0, 0\n");
        return false;
    }
    else
    {
        return true;
    }
}
bool left_650()
{
    float current_time = timer.read();
    if (current_time <= 14)
    {
        controller.setTargetTwist({0, 60, 0});
        printf("0, 40.0, 0\n");
        printf("current_time: %d", (int)current_time);
        return false;
    }
    else
    {
        return true;
    }
}
bool forward_500()
{
    if (backlaser.getOutput1())
    {
        controller.setTargetTwist({70, 0, 0});
        printf("40, 0, 0\n");
        return false;
    }
    else
    {
        return true;
    }
}
bool backward_750()
{
    if (!backlaser.getOutput2())
    {
        controller.setTargetTwist({-100, 0, 0});
        printf("-40, 0, 0\n");
        return false;
    }
    else
    {
        return true;
    }
}
bool left_950()
{
    if (rightlaser.getOutput())
    {
        controller.setTargetTwist({0, 100, 0});
        printf("0, 40, 0");
        return false;
    }
    else
    {
        return true;
    }
}
bool rotate_90()
{
    float current_time = timer.read();
    if (current_time <= 3.4)
    {
        controller.setTargetTwist({0, 0, -1.0});
        printf("0, 0, -1.0\n");
        return false;
    }
    else
    {
        return true;
    }
}
bool wait() {
    float current_time = timer.read();
    if (current_time < 10) {
        stop();
        return false;
    } else {
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
        // aircylinder.off();
#endif
        return true;
    }
}

enum Mode
{
    INIT,
    FORWARD_1700,
    STOP_20_1,
    BACKWARD_500,
    LEFT_650,
    FORWARD_500,
    STOP_20_2,
    BACKWARD_750,
    LEFT_950,
    ROTATE,
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
        if (forward_1400())
        {
            updateMode(Mode::STOP_20_1);
            currentMode = Mode::STOP_20_2;
        }
        break;
    case Mode::STOP_20_1:
        if (stop_20())
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
            updateMode(Mode::STOP_20_2);
            currentMode = Mode::STOP_20_2;
        }
        break;
    case Mode::STOP_20_2:
        if (stop_20())
        {
            updateMode(Mode::BACKWARD_750);
            currentMode = Mode::BACKWARD_750;
        }
        break;
    case Mode::BACKWARD_750:
        if (backward_750())
        {
            updateMode(Mode::LEFT_950);
            currentMode = Mode::LEFT_950;
        }
        break;
    case Mode::LEFT_950:
        if (left_950())
        {
            updateMode(Mode::ROTATE);
            currentMode = Mode::ROTATE;
        }
        break;
    case Mode::ROTATE:
        if (rotate_90())
        {
            updateMode(Mode::WAIT);
            currentMode = Mode::WAIT;
        }
        break;
    case Mode::WAIT:
        if (wait()) {
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
    ThisThread::sleep_for(1000ms);

    while (start_sw == 1)
    {
        ThisThread::sleep_for(100ms);
    }
    
    timer.start();
    
#if TEST
    robot_twist_up(0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
#endif

#if !TEST
    while(1) {
        update();
        printf("right laser: %d\n", rightlaser.getOutput());
        printf("back laser: %d, %d\n", backlaser.getOutput1(), backlaser.getOutput2());
    }
#endif
}
