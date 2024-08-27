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
#include "parameters.hpp"

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

// オムニホイールの配置を設定
float WHEEL_RAD = 100.0; // 車輪の半径
float TREAD_RAD = 100.0; // 中心から車輪までの距離
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD, // 車輪の半径
        .wheel_x = 0.0, // 車輪のx座標
        .wheel_y = TREAD_RAD,  // 車輪のy座標
        .wheel_theta = M_PI // 車輪の角度
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = - M_SQRT3 / 2 * TREAD_RAD,
        .wheel_y = - 0.5 * TREAD_RAD, 
        .wheel_theta = 5 * M_PI / 3
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = + M_SQRT3 / 2 * TREAD_RAD, 
        .wheel_y = - 0.5 * TREAD_RAD,
        .wheel_theta = M_PI / 3
    }
};

// オドメトリとホイールコントローラの設定
Odometry<3> odometry(config, {&encoder_FL, &encoder_FR, &encoder_BL});
WheelController<3> controller(config, {&motor1, &motor2, &motor3});
PIDController robot_pause_pid(0.2,0.0,0.0,20);
Timer timer;
Ticker ticker;

float last_time;
float vx, vy;

enum class Mode {
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

Mode currentMode;
float distanceErrorx = 0.0;
float distanceErrory = 0.0;
float distanceErrortheta = 0.0;
float threshold = 5.0;
float targetDistance = 1700;
float targetAngle;

void robot_twist(float max_v, float accx, float accy, float last_x, float last_y, float target_x, float target_y, float target_theta, float x, float y, float theta) { 
    // target_x単位はmm
    // kp1, kd,ki 0.01くらい

    float middle_x = (target_x+last_x)/2;
    float middle_y = (target_y+last_y)/2;
    float rotations_FL = encoder_FL.getRotations();
    float rotations_FR = encoder_FR.getRotations();
    float rotations_BL = encoder_BL.getRotations();

    float fl = rotations_FL*2.0f*PI*WHEEL_RADIUS;
    float fr = rotations_FR*2.0f*PI*WHEEL_RADIUS;
    float bl = rotations_BL*2.0f*PI*WHEEL_RADIUS;    

    float current_position_x = (fl+fr+bl) /3.0f;
    float current_position_y = (fl+fr+bl) /3.0f;

    if (x <= middle_x && y <= middle_y) {
        printf("1st\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        vx = accx*(current_time);
        vy = accy*(current_time);

        last_time = current_time;
        controller.setTargetTwist({vx, vy, 0});
    } else if (10 < x < middle_x || 10 < y < middle_y ) {
        printf("2nd\n");
        float current_time = timer.read();
        printf("current_time = %d\n", (int)current_time);
        controller.setTargetTwist({robot_pause_pid.calculate(target_x-y), robot_pause_pid.calculate(target_y-y), robot_pause_pid.calculate(target_theta-theta)});
    } else {
        motor1.stop();
        motor2.stop();
        motor3.stop();
    }
}

Pose current_pose;

void forward_1700() {
    robot_twist(2.0, 0, 2.0, 0, 0, 0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void stop_30() {
    // is_moving = false;
    motor1.stop();
    motor2.stop();
    motor3.stop();
    wait_us(30000);
}
void backward_500() {
    // is_moving = true;
    robot_twist(10,0, 2.0, 0, 1700, 0, 1700-500, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void left_650() {
    robot_twist(10,-2.0, 0, 0,1700-500, -650, 1700-500, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void forward_500() {
    robot_twist(10,0, 2.0, -650, 1700-500, -650, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void backward_750() {
    // is_moving = true;
    robot_twist(10,0, 2.0, -650, 1700, -650, 1700-750, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void rotate_90_CW() {
    // ロボット座標系で上から見て時計回り90度回転は-90度回転
    robot_twist(10,0, 0, -650, 1700-750, -650, 1700-750, -90, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void left_950() {
    robot_twist(2,7.0, 0, -650, 1700-750, -650-950, 1700-950, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}
void stop() {
    motor1.stop();
    motor2.stop();
    motor3.stop();
}

void update() {
    // 誤差の計算
    distanceErrorx = targetDistance - current_pose.x;
    distanceErrory = targetDistance - current_pose.y;
    distanceErrortheta = targetDistance - current_pose.theta;

    // アクションの切り替え
    if (abs(distanceErrorx) <= threshold && abs(distanceErrory) <= threshold) {
        switch (currentMode) {
            case Mode::FORWARD_1700:
                    currentMode = Mode::STOP_30_1;

                    break;
                case Mode::STOP_30_1:
                    currentMode = Mode::BACKWARD_500;
                    targetDistance = 500;
                    break;
                case Mode::BACKWARD_500:
                    currentMode = Mode::LEFT_650;
                    targetDistance = 650;
                    break;
                case Mode::LEFT_650:
                    currentMode = Mode::FORWARD_500;
                    targetDistance = 500;
                    break;
                case Mode::FORWARD_500:
                    currentMode = Mode::STOP_30_2;
                    break;
                case Mode::STOP_30_2:
                    currentMode = Mode::BACKWARD_750;
                    targetDistance = 750;
                    break;
                case Mode::BACKWARD_750:
                    currentMode = Mode::ROTATE;
                    targetDistance = 90;
                    break;
                case Mode::ROTATE:
                    currentMode = Mode::LEFT_950;
                    targetDistance = 950;
                    break;
                case Mode::LEFT_950:
                    currentMode = Mode::STOP;
                    break;
                case Mode::STOP:
                    // 何もしない
                    break;
            }
        }

        // 各モードの処理
        switch (currentMode) {
            case Mode::FORWARD_1700:
                forward_1700();
                break;
            case Mode::STOP_30_1:
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
                stop_30();
                break;
            case Mode::BACKWARD_750:
                backward_750();
                break;
            case Mode::ROTATE:
                rotate_90_CW();
                break;
            case Mode::LEFT_950:
                left_950();
                break;
            case Mode::STOP:
                stop();
                break;
    }
}

bool is_moivng = false;

int main() {
    // wait_us(3000);
    // Twist target_twist = ;
    printf("hello\n");
    timer.start();
    
    while (1)
    {
        printf("Hello\n");
        current_pose = odometry.getPose();
        printf("pos: %d, %d, %d\n", (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        printf("difference_x = %d, difference_y = %d, difference_theta = %d\n", (int)current_pose.x, 1700-(int)current_pose.y, (int)current_pose.theta);

        float current_time = timer.read();
        float first_time = 30;

        update();
        
        // ticker.attach(forward_1700, 30000);
        // ticker.attach(stop_30, 30000);
        // ticker.attach(backward_500, 3000);
        // ticker.attach(left_650, 4000);
        // ticker.attach(forward_500, 3000);
        // ticker.attach(stop_30, 30000);
        // ticker.attach(backward_750, 5000);
        // ticker.attach(left_950, 6000);

        // robot_twist(0, 7.0, 0,0, 0, 1700, 0, (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
        wait_us(5000);

        // // 機体が回転したときにロボット座標系がフィールド座標系に対して回転することに注意？
        // // ロボット座標系で上から見て時計回り90度回転は-90度回転
            
    }
    
    // controller.setTargetTwist({0, 0, 0});
    // current_pose = odometry.getPose();
    // printf("x = %d, y = %d, theta = %d\r\n", (int)current_pose.x, (int)current_pose.y, (int)current_pose.theta);
}


