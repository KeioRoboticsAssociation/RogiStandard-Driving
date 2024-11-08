// includeする
#include <mbed.h>

#include "DCMotor/DCMotor.hpp"
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"
#include "OdomWheel/WheelController.hpp"
#include "RogiLinkFlex/UartLink.hpp"
#include "RogiLinkFlex/CommunicationBase.hpp"
#include "Simulator/MotorSimulator.hpp"
#include "pins.hpp"

// encoderの設定
Encoder encoder1(InterruptInPins::MEASURING_ENCODER1_A,
                 DigitalInPins::MEASURING_ENCODER1_B, 2048, 0, false);
Encoder encoder2(InterruptInPins::MEASURING_ENCODER2_A,
                 DigitalInPins::MEASURING_ENCODER2_B, 2048, 0, false);
Encoder encoder3(InterruptInPins::MEASURING_ENCODER3_A,
                 DigitalInPins::MEASURING_ENCODER3_B, 2048, 0, false);

// DCモータの設定
DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);

// モーターコントローラの設定
MotorController motor1(dc1, encoder1);
MotorController motor2(dc2, encoder2);
MotorController motor3(dc3, encoder3);

// オムニホイールの配置を設定
// float WHEEL_RAD = 100.0; // 車輪の半径
// float TREAD_RAD = 100.0; // 中心から車輪までの距離
// std::array<WheelConfig, 3> config = {
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD, // 車輪の半径
//         .wheel_x = 0.0, // 車輪のx座標
//         .wheel_y = TREAD_RAD,  // 車輪のy座標
//         .wheel_theta = M_PI // 車輪の角度
//     },
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD,
//         .wheel_x = - M_SQRT3 / 2 * TREAD_RAD,
//         .wheel_y = - 0.5 * TREAD_RAD,
//         .wheel_theta = 5 * M_PI / 3
//     },
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD,
//         .wheel_x = + M_SQRT3 / 2 * TREAD_RAD,
//         .wheel_y = - 0.5 * TREAD_RAD,
//         .wheel_theta = M_PI / 3
//     }
// };

// // オドメトリとホイールコントローラの設定
// Odometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});
// WheelController<3> controller(config, {&motor1, &motor2, &motor3});

UartLink pc(USBTX, USBRX, 9600);
Subscriber sub<float, float, float>(pc, 1);

Ticker t;

void sub_callback(float _angle1, float _angle2, float _angle3)
{
  motor1.setTargetSpeed((_angle1 - motor1.encoder.getRadians()) / 0.001);
  motor2.setTargetSpeed((_angle2 - motor1.encoder.getRadians()) / 0.001);
  motor3.setTargetSpeed((_angle3 - motor1.encoder.getRadians()) / 0.001);
  t.attach(
      [&]()
      {
        motor1.stop();
        motor2.stop();
        motor3.stop();

        t.dettach();
      })
}

int main()
{
  sub.set_callback(sub_callback);
}
