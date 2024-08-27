#pragma once
#include "Wheel.hpp"
#include "MotorController/MotorController.hpp"
#include <math.h>
#include <array>

class WheelControllerBase {
    public:
        WheelControllerBase() {}
        virtual void setTargetTwist(const Twist twist) = 0;
};

template <int N> // N: 車輪の数
class WheelController {
public:
    WheelController(const std::array<WheelConfig, N>& wheel_configs, const std::array<MotorController*, N>& motors, float max_speed=10000) 
    : motors(motors)
    {
        for (int i = 0; i < N; i++) {
            wheel_vectors[i] = getWheelVector(wheel_configs[i]);
        }

        for (int i = 0; i < N; i++) {
            motors[i]->setTargetSpeed(0);
        }

        this->max_speed = max_speed;
    }

    std::array<float, N> twistToMotorSpeeds(const Twist twist){
        std::array<float, N> speeds;
        float dec_ratio = 1.0; // 速度の減衰比

        for (int i = 0; i < N; i++) {
            speeds[i] = getWheelSpeedRelative(twist, wheel_vectors[i]); // 車輪の速度を計算
            if (fabs(speeds[i]) > max_speed) {
                dec_ratio = fmin(dec_ratio, max_speed / fabs(speeds[i])); // 速度が最大速度を超えた場合、減衰比を更新
            }
        }

        for (int i = 0; i < N; i++) {
            speeds[i] = dec_ratio * speeds[i]; // 速度を減衰
        }

        return speeds;
    }

    // ロボットの目標Twist(ロボット座標)を設定する。
    void setTargetTwist(const Twist twist) {
        auto speeds = twistToMotorSpeeds(twist);

        for (int i = 0; i < N; i++) {
            motors[i]->setTargetSpeed(speeds[i]); // モーターに速度を設定
        }
    }
    

private:
    std::array<WheelVector, N> wheel_vectors;
    float max_speed;
    std::array<MotorController*, N> motors;
};

typedef WheelController<3> ThreeWheelController;
typedef WheelController<4> FourWheelController;

