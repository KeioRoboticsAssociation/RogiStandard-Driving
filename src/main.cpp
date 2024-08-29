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

// encoderの設定
Encoder encoder_FL(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, 1, false);
Encoder encoder_FR(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, 1, false);
Encoder encoder_BL(InterruptInPins::MEASURING_ENCODER3_A, DigitalInPins::MEASURING_ENCODER3_B, 2048, 1, false);
// Encoder encoder_BR(InterruptInPins::MEASURING_ENCODER4_A, DigitalInPins::MEASURING_ENCODER4_B, 2048, 1, false);

// DCモータの設定
DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);

// モーターコントローラの設定
MotorController motor1(dc1, encoder_FL, {0.16, 0.01, 0.01, 20});
MotorController motor2(dc2, encoder_FR, {0.16, 0.01, 0.01, 20});
MotorController motor3(dc3, encoder_BL, {0.16, 0.01, 0.01, 20});

// WTT12L rightlaser(AnalogInPins::RIGHT_WTT12L);
// WTT12L backlaser(AnalogInPins::BACK_WTT12L);

// BNO055 gyrosensor(ConsolePins::CONSOLE_TX, ConsolePins::CONSOLE_RX);

// オムニホイールの配置を設定
float WHEEL_RAD = 100.0; // 車輪の半径
float TREAD_RAD = 100.0; // 中心から車輪までの距離
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD, // 車輪の半径
        .wheel_x = 0.0,            // 車輪のx座標
        .wheel_y = TREAD_RAD,      // 車輪のy座標
        .wheel_theta = M_PI        // 車輪の角度
    },
    WheelConfig{
        .wheel_radius = WHEEL_RAD,
        .wheel_x = -M_SQRT3 / 2 * TREAD_RAD,
        .wheel_y = -0.5 * TREAD_RAD,
        .wheel_theta = 5 * M_PI / 3},
    WheelConfig{
        .wheel_radius = WHEEL_RAD,
        .wheel_x = +M_SQRT3 / 2 * TREAD_RAD,
        .wheel_y = -0.5 * TREAD_RAD,
        .wheel_theta = M_PI / 3}};

// オドメトリとホイールコントローラの設定
Odometry<3> odometry(config, {&encoder_FL, &encoder_FR, &encoder_BL});
WheelController<3> controller(config, {&motor1, &motor2, &motor3});
PIDController robot_pose_pid(0.7, 0.0, 0.0, 20);
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

    float rotations_FL = encoder_FL.getRotations();
    float rotations_FR = encoder_FR.getRotations();
    float rotations_BL = encoder_BL.getRotations();

    float fl = rotations_FL * 2.0f * PI * WHEEL_RADIUS;
    float fr = rotations_FR * 2.0f * PI * WHEEL_RADIUS;
    float bl = rotations_BL * 2.0f * PI * WHEEL_RADIUS;

    if (((last_x-x) <= (x-target_x)) || ((y-last_y) <= (target_y-y))) {
        printf("1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx * (current_time);
        vy = accy * (current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
        printf("last_x: %d, x: %d, last_y: %d, y: %d\n", (int)last_x, (int)x, (int)last_y, (int)y);
    } else if (((0 < (x-target_x)) && ((x-target_x) < (last_x-x))) || ((0 < (target_y-y)) && ((target_y-y) < (y-last_y)))) {
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
    float rotations_FL = encoder_FL.getRotations();
    float rotations_FR = encoder_FR.getRotations();
    float rotations_BL = encoder_BL.getRotations();

    float fl = rotations_FL * 2.0f * PI * WHEEL_RADIUS;
    float fr = rotations_FR * 2.0f * PI * WHEEL_RADIUS;
    float bl = rotations_BL * 2.0f * PI * WHEEL_RADIUS;

    if (((last_x-x) <= (x-target_x)) || ((last_y-y) < (y-target_y))) {
        printf("2-1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx * (current_time);
        vy = accy * (current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
    } else if (((0 < (x-target_x)) && ((x-target_x) < (last_x-x))) || (0 < (y-target_y)) && ((y-target_y) < (last_y-y))) {
        printf("2-2nd\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        controller.setTargetTwist({robot_pose_pid.calculate(target_x - y), robot_pose_pid.calculate(target_y - y), robot_pose_pid.calculate(target_theta - theta)});
    } else {
        printf("2-3rd\n");
        movement_id++;
        motor1.stop();
        motor2.stop();
        motor3.stop();
    }
}

float acc = 10.0;

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
    robot_twist_up(10, -acc, 0, 0, 1700 - 500, -650, 1700 - 500, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
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
    robot_twist_up(10, -acc, 0, -650, 1700 - 750, -650 - 950, 1700 - 950, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void stop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();
}


enum class Mode
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

float distanceError = 0.0;
float thetaError = 0.0;
float threshold = 100.0;
float targetx = 0;
float targety = 1700;
float targetAngle;

void update()
{
    // 誤差の計算
    distanceError = sqrt(pow((targetx - current_pose.x), 2.0)+pow((targety - current_pose.y), 2.0));
    thetaError = abs(targetAngle - current_pose.theta);

    // アクションの切り替え
    if (abs(distanceError) <= threshold)
    {
        switch (currentMode)
        {
        case Mode::FORWARD_1700:
            updateMode(Mode::STOP_30_1);
            currentMode = Mode::STOP_30_1;
            // テストのため仮
            targety = 1700-500;
            break;
        case Mode::STOP_30_1:
            updateMode(Mode::BACKWARD_500);
            currentMode = Mode::BACKWARD_500;
            targety = 1700-500;
            break;
        case Mode::BACKWARD_500:
            updateMode(Mode::LEFT_650);
            currentMode = Mode::LEFT_650;
            targetx = -650;
            break;
        case Mode::LEFT_650:
            updateMode(Mode::FORWARD_500);
            currentMode = Mode::FORWARD_500;
            targety = 1700;
            break;
        case Mode::FORWARD_500:
            updateMode(Mode::STOP_30_2);
            currentMode = Mode::STOP_30_2;
            break;
        case Mode::STOP_30_2:
            updateMode(Mode::BACKWARD_750);
            currentMode = Mode::BACKWARD_750;
            targety = 1700-750;
            break;
        case Mode::BACKWARD_750:
            updateMode(Mode::ROTATE);
            currentMode = Mode::ROTATE;
            targetAngle = 90;
            break;
        case Mode::ROTATE:
            updateMode(Mode::LEFT_950);
            currentMode = Mode::LEFT_950;
            targetx = -650-950;
            break;
        case Mode::LEFT_950:
            updateMode(Mode::STOP);
            currentMode = Mode::STOP;
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
        backward_500();
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

        float current_time = timer.read();

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
