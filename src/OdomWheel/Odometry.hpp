#pragma once

#include "Wheel.hpp"
#include "Encoder/Encoder.hpp"

// 警告を無視する
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"

#include "Eigen/Dense.h"


template <int N>
class Odometry {
    static_assert(N >= 2, "N must be greater than 2.");
    public:
        Odometry(const std::array<WheelConfig, N>& wheel_configs, const std::array<Encoder*, N>& encoders, std::chrono::microseconds update_interval = std::chrono::microseconds(5000)) 
            : encoders(encoders)
        {
            wheel_vectors_inv = getWheelVectorInv(wheel_configs);
            ticker.attach(callback(this, &Odometry::update), update_interval); // オドメトリの更新周期を設定
            setPose({0.0, 0.0, 0.0});
            last_encoder_counts.fill(0);
            this->update_interval = update_interval;
        }

        static std::array<WheelVectorInv, N> getWheelVectorInv(const std::array<WheelConfig, N>& wheel_configs) {
            Eigen::MatrixXd wheel_matrix(N, 3); // 車輪のベクトルを格納する行列
            for (int i = 0; i < N; i++) {
                WheelVector wheel_vector = getWheelVector(wheel_configs.at(i));
                wheel_matrix(i, 0) = wheel_vector.kx;
                wheel_matrix(i, 1) = wheel_vector.ky;
                wheel_matrix(i, 2) = wheel_vector.ktheta;
            }

            Eigen::MatrixXd wheel_matrix_inv(3, N); // 逆行列 (N>=4のときMoore-Penroseの疑似逆行列)
            if (N == 3) {
                wheel_matrix_inv = wheel_matrix.inverse();
            } else {
                wheel_matrix_inv = wheel_matrix.transpose() * (wheel_matrix * wheel_matrix.transpose()).inverse();
            }

            std::array<WheelVectorInv, N> wheel_vectors_inv; // 車輪のベクトルの逆行列
            for (int i = 0; i < N; i++) {
                wheel_vectors_inv.at(i).kx = wheel_matrix_inv(0, i);
                wheel_vectors_inv.at(i).ky = wheel_matrix_inv(1, i);
                wheel_vectors_inv.at(i).ktheta = wheel_matrix_inv(2, i);
            }

            return wheel_vectors_inv;
        }

        void update(){
            std::array<int, N> encoder_counts;
            for (int i = 0; i < N; i++) {
                encoder_counts.at(i) = encoders.at(i)->getCount();
            }

            float delta_x = 0.0;
            float delta_y = 0.0;
            float delta_theta = 0.0;

            for (int i = 0; i < N; i++) {
                float encoder_delta = encoders.at(i)->countToRotations(encoder_counts.at(i) - last_encoder_counts.at(i));
                last_encoder_counts.at(i) = encoder_counts.at(i); // 最後のエンコーダーのカウントを更新
                delta_x += encoder_delta * wheel_vectors_inv.at(i).kx;
                delta_y += encoder_delta * wheel_vectors_inv.at(i).ky;
                delta_theta += encoder_delta * wheel_vectors_inv.at(i).ktheta;
            }

            updatePose(delta_x, delta_y, delta_theta);
        }

        Pose getPose() {
            return pose;
        }

        void setPose(Pose new_pose) {
            pose = new_pose;
        }

        void setTheta(float theta) {
            pose.theta = theta;
        }

        int DEBUG_ = 0;

        std::chrono::microseconds getUpdateInterval() {
            return update_interval;
        }

        


    private:
        std::array<Encoder*, N> encoders;
        std::array<int, N> last_encoder_counts;
        std::array<WheelVectorInv, N> wheel_vectors_inv;
        Pose pose;
        Ticker ticker;

        std::chrono::microseconds update_interval;

        void updatePose(float delta_x, float delta_y, float delta_theta) {
            // ロボット座標系からフィールド座標系への変換を含む位置と姿勢の更新
            float delta_x_abs = delta_x * cos(pose.theta + delta_theta / 2) - delta_y * sin(pose.theta + delta_theta / 2); // x方向の移動量 (フィールド座標系)
            float delta_y_abs = delta_x * sin(pose.theta + delta_theta / 2) + delta_y * cos(pose.theta + delta_theta / 2); // y方向の移動量 (フィールド座標系)

            pose.x += delta_x_abs;
            pose.y += delta_y_abs;
            pose.theta += delta_theta;
        }
};

typedef Odometry<3> ThreeWheelOdometry;
typedef Odometry<4> FourWheelOdometry;

#pragma GCC diagnostic pop

