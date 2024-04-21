#if 1
#include <mbed.h>
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"
#include "OdomWheel/WheelController.hpp"
#include "DCMotor/DCMotor.hpp"
#include "pins.hpp"
#include "Simulator/MotorSimulator.hpp"

Encoder encoder1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, 0, false);
Encoder encoder2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, 0, false);
Encoder encoder3(InterruptInPins::MEASURING_ENCODER3_A, DigitalInPins::MEASURING_ENCODER3_B, 2048, 0, false);

DCMotor dc1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, 0);
DCMotor dc2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR, 0);
DCMotor dc3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, 0);

MotorController motor1(dc1, encoder1, PIDGain({0.1, 0.01, 0.01, 100}));
MotorController motor2(dc2, encoder2, PIDGain({0.1, 0.01, 0.01, 100}));
MotorController motor3(dc3, encoder3, PIDGain({0.1, 0.01, 0.01, 100}));

float WHEEL_RAD = 100.0;
float TREAD_RAD = 100.0;
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = 0.0, 
        .wheel_y = TREAD_RAD, 
        .wheel_theta = M_PI
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

Odometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});
WheelController<3> controller(config, {&motor1, &motor2, &motor3});



int main() {
    controller.setTargetTwist({1000.0, 0.0, 0.0});
    odometry.setPose({0.0,0.0,0.0});
    while (1) {
        int a = int(60 * motor1.getTargetSpeed());
        int b = int(60 * motor2.getTargetSpeed());
        int c = int(60 * motor3.getTargetSpeed());
        printf("%d, %d, %d\n", a, b, c);
        ThisThread::sleep_for(100ms);
        encoder1.addCount(0);
        encoder2.addCount(0);
        encoder3.addCount(0);
        auto pose = odometry.getPose();
        //printf("%d\n", odometry.DEBUG_);
        //printf("%d, %d, %d\n", encoder1.getCount(), encoder2.getCount(), encoder3.getCount());
        printf("x: %d, y: %d, theta: %d\n", int(1000 * pose.x), int(1000 * pose.y), int(1000 * pose.theta));
    }
}


#endif

#if 0

#include <mbed.h>
#include "OdomWheel/Odometry.hpp"

Encoder encoder1(D2, D3, 2048, 0, false);
Encoder encoder2(D4, D5, 2048, 0, false);

std::array<WheelConfig, 2> config = {
    WheelConfig{100.0, 100.0, 0.0, M_PI / 4}, 
    WheelConfig{100.0, -100.0, 0.0, 2 * M_PI / 4}
};

TwoWheelOdometry odometry(config, {&encoder1, &encoder2});

int main() {
    
}




#endif


