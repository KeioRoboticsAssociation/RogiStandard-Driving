#include "MotorController.hpp"

MotorController::MotorController(DCMotor& motor, Encoder& encoder, PIDGain& pid_gain, float max_duty)
    : motor(motor), encoder(encoder), pid_controller(pid_gain)
{
    ticker.attach(callback(this, &MotorController::loop), 1000ms / pid_controller.getFrequency());
    this->max_duty = max_duty; //最大デューティ比を設定
}

void MotorController::loop()
{
    // 速度の更新
    float current_position = encoder.getRotations(); //現在の位置を取得
    current_rps = (current_position - last_position) * pid_controller.getFrequency(); //現在の速度を取得
    last_position = current_position;

    // PID制御
    float output = pid_controller.calculate(target_rps - current_rps);
    // デューティ比の制限
    if (output > max_duty) {
        output = max_duty;
        pid_controller.reset(); //最大速度に達したら積分をリセット
    } else if (output < -max_duty) {
        output = -max_duty;
        pid_controller.reset(); //最大速度に達したら積分をリセット
    }
    // モーターへの出力
    motor.setDuty(output);
}

void MotorController::setTargetSpeed(float target_rps)
{
    this->target_rps = target_rps;
}

void MotorController::stop()
{
    motor.setDuty(0);
}

float MotorController::getSpeed()
{
    return current_rps;
}


