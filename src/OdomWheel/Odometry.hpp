#pragma once
#include "Wheel.hpp"
#include "Encoder/Encoder.hpp"
#include "Eigen/Dense.h"

class OdomBase {
    public:
        OdomBase() {}

        virtual Pose getPose() = 0;
        virtual void setPose(Pose pose) = 0;
        virtual void setTheta(float theta) = 0;

};

// エンコーダーのみを使ったオドメトリ
template <int N> // N: 車輪の数
class WheelOdometry : public OdomBase {
public:
    WheelOdometry(const std::array<WheelConfig, N>& wheel_configs, const std::array<Encoder*, N>& encoders, std::chrono::microseconds update_interval = std::chrono::microseconds(5000))
        : encoders(encoders) {
        std::array<WheelVector, N> wheel_vectors;
        for (int i = 0; i < N; i++) {
            wheel_vectors[i] = getWheelVector(wheel_configs[i]);
        }

        // wheel_vectorsをEigen::MatrixXdに変換
        Eigen::MatrixXd wheel_matrix(N, 3);
        for (int i = 0; i < N; i++) {
            wheel_matrix(i, 0) = wheel_vectors[i].kx;
            wheel_matrix(i, 1) = wheel_vectors[i].ky;
            wheel_matrix(i, 2) = wheel_vectors[i].ktheta;
        }

        // 逆行列 (N>=4のときMoore-Penroseの疑似逆行列) を計算
        if (N == 3) {
            wheel_matrix_inv = wheel_matrix.inverse();
        } else {
            wheel_matrix_inv = wheel_matrix.transpose() * (wheel_matrix * wheel_matrix.transpose()).inverse();
        }

        ticker.attach(callback(this, &WheelOdometry::update), update_interval); // オドメトリの更新周期を設定
    }

    void update() {
        std::array<int, N> encoder_counts;
        for (int i = 0; i < N; i++) {
            encoder_counts[i] = encoders[i]->getCount();
        }

        Eigen::VectorXd enc_delta_vec(N);
        for (int i = 0; i < N; i++) {
            enc_delta_vec(i) = encoders[i]->countToRotations(encoder_counts[i] - last_encoder_counts[i]);
            last_encoder_counts[i] = encoder_counts[i]; // 最後のエンコーダーのカウントを更新
        }

        Eigen::VectorXd pose_delta_vec = wheel_matrix_inv * enc_delta_vec; // 位置と姿勢の変化量を計算
        float delta_x = pose_delta_vec(0); // x方向の移動量 (ロボット座標系)
        float delta_y = pose_delta_vec(1); // y方向の移動量 (ロボット座標系)
        float delta_theta = pose_delta_vec(2); // 回転量

        // 位置と姿勢を更新
        updatePose(delta_x, delta_y, delta_theta);
    }

    Pose getPose() override {
        return pose;
    }

    void setPose(Pose new_pose) override {
        pose = new_pose;
    }

    void setTheta(float theta) override {
        pose.theta = theta;
    }

private:
    void updatePose(float delta_x, float delta_y, float delta_theta) {
        // ロボット座標系からフィールド座標系への変換を含む位置と姿勢の更新
        float delta_x_abs = delta_x * cos(pose.theta + delta_theta / 2) - delta_y * sin(pose.theta + delta_theta / 2); // x方向の移動量 (フィールド座標系)
        float delta_y_abs = delta_x * sin(pose.theta + delta_theta / 2) + delta_y * cos(pose.theta + delta_theta / 2); // y方向の移動量 (フィールド座標系)

        pose.x += delta_x_abs;
        pose.y += delta_y_abs;
        pose.theta += delta_theta;
    }

    Eigen::MatrixXd wheel_matrix_inv; // 車輪のベクトルの逆行列
    std::array<Encoder*, N> encoders;
    Ticker ticker;
    std::array<int, N> last_encoder_counts = {}; // 初期化
    Pose pose;
};

template <>
class WheelOdometry<2> : public OdomBase { 
    public: 
        WheelOdometry(const std::array<WheelConfig, 2>& wheel_configs, const std::array<Encoder*, 2>& encoders, std::chrono::microseconds update_interval = std::chrono::microseconds(5000)) 
        {
            // エンコーダーと車輪の設定を取得
            this->encoders = encoders;

            // 基準点を取得 (測定輪の延長線の交点)
            float sin_theta0 = sin(wheel_configs[0].wheel_theta);
            float cos_theta0 = cos(wheel_configs[0].wheel_theta);
            float sin_theta1 = sin(wheel_configs[1].wheel_theta);
            float cos_theta1 = cos(wheel_configs[1].wheel_theta);

            // 車輪の中心から直線までの距離を計算
            float r0 = -wheel_configs[0].wheel_x * sin_theta0 + wheel_configs[0].wheel_y * cos_theta0; // 直線までの符号付き距離
            float r1 = -wheel_configs[1].wheel_x * sin_theta1 + wheel_configs[1].wheel_y * cos_theta1; // 直線までの符号付き距離

            float det = cos_theta0 * sin_theta1 - sin_theta0 * cos_theta1; // 逆行列の計算に使用

            if (det == 0) {
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Invalid wheel configuration");
            }

            // 逆行列の計算で基準点を求める
            ref_x = (r0 * cos_theta1 - r1 * cos_theta0) / det;
            ref_y = (r0 * sin_theta1 - r1 * sin_theta0) / det;

            // 回転数から位置を計算するための係数
            kx_0 = sin_theta1 / det;
            ky_0 = -cos_theta1 / det;
            kx_1 = -sin_theta0 / det;
            ky_1 = cos_theta0 / det;
            
            ticker.attach(callback(this, &WheelOdometry::update), update_interval); // オドメトリの更新周期を設定
        }

        void update() {
            // エンコーダーのカウントを取得
            float count0 = encoders[0]->getCount();
            float count1 = encoders[1]->getCount();

            // エンコーダーのカウントから回転量を計算
            float delta0 = encoders[0]->countToRotations(count0 - last_encoder_counts[0]);
            float delta1 = encoders[1]->countToRotations(count1 - last_encoder_counts[1]);

            // エンコーダーのカウントを更新
            last_encoder_counts[0] = count0;
            last_encoder_counts[1] = count1;

            // 位置と姿勢の変化量を計算
            float delta_x = kx_0 * delta0 + kx_1 * delta1;
            float delta_y = ky_0 * delta0 + ky_1 * delta1;

            // フィールド座標系に変換する。
            float delta_x_abs = delta_x * cos(theta) - delta_y * sin(theta); // x方向の移動量 (フィールド座標系)
            float delta_y_abs = delta_x * sin(theta) + delta_y * cos(theta); // y方向の移動量 (フィールド座標系)

            // 位置と姿勢を更新
            x += delta_x_abs;
            y += delta_y_abs;
        }

        // 位置と姿勢を取得
        Pose getPose() override {
            return Pose{
                x: x - ref_x * cos(theta) + ref_y * sin(theta),
                y: y - ref_x * sin(theta) - ref_y * cos(theta),
                theta: theta
            };
        }

        // 位置と姿勢を設定
        void setPose(Pose pose) override {
            x = pose.x + ref_x * cos(pose.theta) - ref_y * sin(pose.theta);
            y = pose.y + ref_x * sin(pose.theta) + ref_y * cos(pose.theta);
            theta = pose.theta;
        }

        // 姿勢を設定。つねにジャイロセンサで姿勢を補正しつづける必要がある。
        void setTheta(float theta) override {
            this->theta = theta;
        }

    private:
        std::array<Encoder*, 2> encoders;

        Ticker ticker;
        int last_encoder_counts[2];

        float ref_x; // 基準点のx座標
        float ref_y; // 基準点のy座標

        float x; // 位置(x [mm])
        float y; // 位置(y [mm])
        float theta; // 姿勢(rad)

        float kx_0;
        float ky_0;
        float kx_1;
        float ky_1;
};

// 3輪のオドメトリ
typedef WheelOdometry<3> ThreeWheelOdometry;

// 2輪のオドメトリ(2輪の場合はジャイロセンサと組み合わせる必要がある。)
typedef WheelOdometry<2> TwoWheelOdometry;
