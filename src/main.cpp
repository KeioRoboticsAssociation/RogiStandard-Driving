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
#include "BNO055/BNO055.hpp"
#include "parameters.hpp"
#include "settings.hpp"

#if !PERFORMANCE_ROBOT
// 測定輪encoderの設定
Encoder encoder_1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, 1, false);
Encoder encoder_2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, 1, false);
Encoder encoder_3(InterruptInPins::MEASURING_ENCODER3_A, DigitalInPins::MEASURING_ENCODER3_B, 2048, 1, false);
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
DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);
#endif

#if PERFORMANCE_ROBOT
DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);
DCMotor dc4(PwmOutPins::MOTOR4_PWM, DigitalOutPins::MOTOR4_DIR, 0);
#endif

// モーターコントローラの設定
#if !PERFORMANCE_ROBOT
MotorController motor1(dc1, encoder_1, {0.16, 0.01, 0.01, 20});
MotorController motor2(dc2, encoder_2, {0.16, 0.01, 0.01, 20});
MotorController motor3(dc3, encoder_3, {0.16, 0.01, 0.01, 20});
#endif

#if PERFORMANCE_ROBOT
MotorController motor1(dc1, encoder_FL, {0.16, 0.01, 0.01, 20})
MotorController motor2(dc2, encoder_FR, {0.16, 0.01, 0.01, 20})
MotorController motor3(dc3, encoder_BL, {0.16, 0.01, 0.01, 20})
MotorController motor4(dc4, encoder_BR, {0.16, 0.01, 0.01, 20})
#endif

#if USE_LASER_WTT12L
WTT12L rightlaser(AnalogInPins::RIGHT_WTT12L);
WTT12L backlaser(AnalogInPins::BACK_WTT12L);
#endif

#if USE_GYROSENSOR_BNO055
BNO055 gyrosensor(ConsolePins::CONSOLE_TX, ConsolePins::CONSOLE_RX);
#endif

// オムニホイールの配置を設定
#if !PERFORMANCE_ROBOT
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RADIUS, // 車輪の半径
        .wheel_x = 0.0,            // 車輪のx座標
        .wheel_y = TRED_RADIUS,      // 車輪のy座標
        .wheel_theta = M_PI        // 車輪の角度
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
std::array<WheelConfig, 2> measuring_config = {
    WheelConfig{ // x方向
        .wheel_radius = WHEEL_RADIUS, // 車輪の半径
        .wheel_x = +SQRT2 / 2 * TRED_RADIUS,      // 車輪のx座標
        .wheel_y = +SQRT2 / 2 * TRED_RADIUS,      // 車輪のy座標
        .wheel_theta = M_PI / 4 * 3       // 車輪の角度
    },
    WheelConfig{ // y方向
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = -SQRT2 / 2 * TRED_RADIUS,
        .wheel_y = +SQRT2 / 2 * TRED_RADIUS,
        .wheel_theta = M_PI / 4}};
std::array<WheelConfig, 4> config = {
    WheelConfig{ // FL
        .wheel_radius = WHEEL_RADIUS, // 車輪の半径
        .wheel_x = +SQRT2 / 2 * TRED_RADIUS,      // 車輪のx座標
        .wheel_y = +SQRT2 / 2 * TRED_RADIUS,      // 車輪のy座標
        .wheel_theta = M_PI / 4 * 3       // 車輪の角度
    },
    WheelConfig{ // FR
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = -SQRT2 / 2 * TRED_RADIUS,
        .wheel_y = +SQRT2 / 2 * TRED_RADIUS,
        .wheel_theta = M_PI / 4},
    WheelConfig{ // BL
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = +SQRT2 / 2 * TRED_RADIUS,
        .wheel_y = -SQRT2 / 2 * TRED_RADIUS,
        .wheel_theta = M_PI / 4 * 5},
    WheelConfig{ // BR
        .wheel_radius = WHEEL_RADIUS,
        .wheel_x = -SQRT2 / 2 * TRED_RADIUS,
        .wheel_y = -SQRT2 / 2 * TRED_RADIUS,
        .wheel_theta = M_PI / 4 * 7}};
#endif

// オドメトリとホイールコントローラの設定
#if !PERFORMANCE_ROBOT
Odometry<3> odometry(config, {&encoder_1, &encoder_2, &encoder_3});
WheelController<3> controller(config, {&motor1, &motor2, &motor3});
#endif
#if PERFORMANCE_ROBOT
Odometry<2> odometry(measuring_config, {&encoder_1, &encoder_2});
WheelController<4> controller(config, {&motor1, &motor2, &motor3, &encoder_4});
#endif
PIDController robot_pose_pid(0.5, 0.0, 0.0, 20);
Timer timer;
Ticker ticker;

float last_time;
float vx, vy;

Pose current_pose;
int movement_id = 0;

void robot_twist_up(float max_v, float accx, float accy, float last_x, float last_y, float target_x, float target_y, float target_theta, float x, float y, float theta)
{
    // target_x単位はmm
    // kp1, kd,ki 0.01くらい

    if (((y-last_y) <= (target_y-y))) {
        printf("1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx * (current_time);
        vy = accy * (current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
        printf("last_x: %d, last_y: %d, x: %d, y: %d, target_x: %d, target_y: %d\n", (int)last_x, (int)last_y, (int)x, (int)y, (int)target_x, (int)target_y);
        
    } else if (((0 < (target_y-y)) && ((target_y-y) < (y-last_y)))) {
        printf("2nd\n");
        
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        
        controller.setTargetTwist({robot_pose_pid.calculate(target_x - x), robot_pose_pid.calculate(target_y - y), robot_pose_pid.calculate(target_theta - theta)});
    } else {
        printf("3rd\n");
        printf("last_x: %d, x: %d, last_y: %d, y: %d\n", (int)last_x, (int)x, (int)last_y, (int)y);
        
        movement_id++;
        motor1.stop();
        motor2.stop();
        motor3.stop();
    }
}

void robot_twist_down(float max_v, float accx, float accy, float last_x, float last_y, float target_x, float target_y, float target_theta, float x, float y, float theta)
{
    
    if (((last_y-y) < (y-target_y))) {
        printf("2-1st\n");
        
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        
        vx = accx * (current_time);
        vy = accy * (current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
        printf("vx = %d, vy = %d\n", (int)vx, (int)vy);
        
    } else if ((0 < (y-target_y)) && ((y-target_y) < (last_y-y))) {
        printf("2-2nd\n");
        
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        
        controller.setTargetTwist({robot_pose_pid.calculate(target_x - x), robot_pose_pid.calculate(target_y - y), robot_pose_pid.calculate(target_theta - theta)});
    } else {
        printf("2-3rd\n");
        
        movement_id++;
        motor1.stop();
        motor2.stop();
        motor3.stop();
    }
}

void robot_twist_left(float max_v, float accx, float accy, float last_x, float last_y, float target_x, float target_y, float target_theta, float x, float y, float theta)
{
    
    if (((last_x-x) <= (x-target_x))) {
        printf("3-1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx * (current_time);
        vy = accy * (current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
        printf("last_x: %d, last_y: %d, x: %d, y: %d, target_x: %d, target_y: %d\n", (int)last_x, (int)last_y, (int)x, (int)y, (int)target_x, (int)target_y);
    } else if (((0 < (x-target_x)) && ((x-target_x) < (last_x-x)))) {
        printf("3-2nd\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        controller.setTargetTwist({robot_pose_pid.calculate(target_x - x), robot_pose_pid.calculate(target_y - y), robot_pose_pid.calculate(target_theta - theta)});
    } else {
        printf("3-3rd\n");
        printf("last_x: %d, x: %d, last_y: %d, y: %d\n", (int)last_x, (int)x, (int)last_y, (int)y);
        
        movement_id++;
        motor1.stop();
        motor2.stop();
        motor3.stop();
    }
}

float acc = 10.0;

float distanceError = 0.0;
float thetaError = 0.0;
float threshold = 200.0;
float targetx = 0;
float targety = 1700;
float targetAngle;

void forward_1700()
{
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_up(10.0, 0, 10.0, 0, 0, 0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void scanRight() {
    #if USE_LASER_WTT12L
    bool isclose = rightlaser.whetherclose(0.075); // 75mm
    if (!isclose) { 
        controller.setTargetTwist({robot_pose_pid.calculate(0.05), 0, 0});
    } else {
        return;
    }
    #endif
    wait_us(2000);
}
void stop_30()
{
    // is_moving = false;
    controller.setTargetTwist({0,0,0});
    wait_us(30000);
}
void backward_500()
{
    // is_moving = true;
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_down(10, 0, -acc, 0, 1700, 0, 1700 - 500, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void left_650()
{
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_left(10, -acc, 0, 0, 1700 - 500, -650, 1700 - 500, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void forward_500()
{
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_up(10, 0, acc, -650, 1700 - 500, -650, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void backward_750()
{
    // is_moving = true;
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_down(10, 0, -acc, -650, 1700, -650, 1700 - 750, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void scanBack()
{
    #if USE_LASER_WTT12L
    bool ifclose = backlaser.whetherclose(0.95); // 950mm
    if (!ifclose) {
        return;
    } else {

    }
    #endif
}
void rotate_90_CW()
{
    // ロボット座標系で上から見て時計回り90度回転は-90度回転
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(- M_PI/2);
    #endif
    robot_twist_up(10, 0, 0, -650, 1700 - 750, -650, 1700 - 750, -M_PI/2, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void left_950()
{
    #if USE_GYROSENSOR_BNO055
    gyrosensor.setRadians(0);
    #endif
    robot_twist_left(10, -acc, 0, -650, 1700 - 750, -650 - 950, 1700 - 950, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void stop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();
}


enum Mode
{
    FORWARD_1700,
    STOP_30_1,
    BACKWARD_500,
    LEFT_650,
    FORWARD_500,
    STOP_30_2,
    BACKWARD_750,
    ROTATE,
    LEFT_950,
    STOP
};

Mode currentMode = Mode::FORWARD_1700;
Mode previous_mode = Mode::FORWARD_1700;

void updateMode(Mode new_mode) {
    if (new_mode != currentMode) {
        // モードが切り替わったときのみタイマーをリセット
        timer.reset();
        timer.start();
        
        // モードが切り替わった後に更新
        previous_mode = currentMode;
        currentMode = new_mode;
    }
}

#if USE_LASER_WTT12L
void adjustPositionToWall() {
    while (1) {        
        if (rightlaser.whetherclose(0.05) == true) {
            // 現在の距離が目標より小さい場合、左に移動
            controller.setTargetTwist({-0.5, 0, 0});
        } else if (rightlaser.whetherclose(0.075) == true) {
            // 現在の距離が目標より大きい場合、右に移動
            controller.setTargetTwist({0.5, 0, 0});
        } else {
            current_pose.x = 0;
            stop();
            printf("Position adjusted: Distance to wall = %.2f mm\n", 0.075-current_pose.x);
            break;
        }
        wait_us(100); // 100ms
    }
}
#endif

void update()
{ 
    // 誤差の計算
    distanceError = sqrt(pow((targetx - current_pose.x), 2.0)+pow((targety - current_pose.y), 2.0));
    thetaError = abs(targetAngle - current_pose.theta);

    // アクションの切り替え
    if (abs(distanceError) <= threshold)
    {
        switch (currentMode)        {
        case Mode::FORWARD_1700:
            updateMode(Mode::STOP_30_1);
            currentMode = Mode::STOP_30_1;
            movement_id = 1;
            break;
        case Mode::STOP_30_1:
            updateMode(Mode::BACKWARD_500);
            currentMode = Mode::BACKWARD_500;
            movement_id = 2;
            targety = 1700-500;
            break;
        case Mode::BACKWARD_500:
            updateMode(Mode::LEFT_650);
            currentMode = Mode::LEFT_650;
            movement_id =3;
            targetx = -650;
            break;
        case Mode::LEFT_650:
            updateMode(Mode::FORWARD_500);
            currentMode = Mode::FORWARD_500;
            movement_id =4;
            targety = 1700;
            break;
        case Mode::FORWARD_500:
            updateMode(Mode::STOP_30_2);
            currentMode = Mode::STOP_30_2;
            movement_id =5;
            break;
        case Mode::STOP_30_2:
            updateMode(Mode::BACKWARD_750);
            currentMode = Mode::BACKWARD_750;
            movement_id = 6;
            targety = 1700-750;
            break;
        case Mode::BACKWARD_750:
            updateMode(Mode::ROTATE);
            currentMode = Mode::ROTATE;
            movement_id =7;
            targetAngle = 90;
            break;
        case Mode::ROTATE:
            updateMode(Mode::LEFT_950);
            currentMode = Mode::LEFT_950;
            movement_id =8;
            targetx = -650-950;
            break;
        case Mode::LEFT_950:
            updateMode(Mode::STOP);
            currentMode = Mode::STOP;
            movement_id =9;
            break;
        case Mode::STOP:
            // 何もしない
            break;
        }
    }

    // 各モードの処理
    switch (currentMode)
    {
    case Mode::FORWARD_1700:
        forward_1700();
        break;
    case Mode::STOP_30_1:
        #if USE_LASER_WTT12L
        scanRight();
        #endif
        stop_30();
        break;
    case Mode::BACKWARD_500:
        backward_500();
        break;
    case Mode::LEFT_650:
        left_650();
        break;
    case Mode::FORWARD_500:
        forward_500();
        break;
    case Mode::STOP_30_2:
        #if USE_LASER_WTT12L
        scanRight();
        #endif
        stop_30();
        break;
    case Mode::BACKWARD_750:
        backward_750();
        break;
    case Mode::ROTATE:
        #if USE_LASER_WTT12L
        scanBack();
        #endif
        rotate_90_CW();
        break;
    case Mode::LEFT_950:
        #if USE_GYROSENSOR_BNO055
        gyrosensor.reset();
        #endif
        left_950();
        break;
    case Mode::STOP:
        stop();
        break;
    }
}

bool is_moivng = false;

int main()
{
    // wait_us(3000);
    // Twist target_twist = ;
    printf("hello\n");
    timer.start();

    while (1)
    {
        printf("Hello\n");
        current_pose = odometry.getPose();
        printf("pos: %d, %d, %d\n", (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        printf("difference_x = %d, difference_y = %d, difference_theta = %d\n", (int)current_pose.x, 1700 - (int)current_pose.y, (int)current_pose.theta);

        #if TEST
        robot_twist_up(10.0, 0, 10.0, 0, 0, 0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        #endif

        #if !TEST
        update();
        #endif

        // wait_us(5000);

        // 機体が回転したときにロボット座標系がフィールド座標系に対して回転することに注意？
        // ロボット座標系で上から見て時計回り90度回転は-90度回転
    }
}
