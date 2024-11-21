#include "C620.hpp"

// コンストラクタ
C620::C620(CAN &_can, unsigned int _motor_id, bool _direction)
    : can(_can)
    , motor_id(_motor_id)
    , direction(_direction)
    , speed(0)
    , torque(0)
    , cumulative_angle(0)
    , last_angle(0)
    , temperature(0)
    , target_speed(0)
    , gain({100, 0, 0, 1000})
    , integral(0.0f)
    , prev_error(0)
    , control_loop_active(true)
{
    ctrl_period = 1.0f / gain.frequency;

    // スレッドで制御ループを開始
    t.attach(callback(this, &C620::control_loop), std::chrono::milliseconds((int)(1000 * this->ctrl_period)) );

    can.attach(callback(this, &C620::update_feedback), CAN::RxIrq);
}

C620::~C620() {
    t.detach();
}

void C620::set_current(int16_t current) {
    const int motorIndex = (motor_id - 1) * 2;

    uint8_t data[8] = {0};
    data[motorIndex] = (current >> 8) & 0xFF; // 電流の上位バイト
    data[motorIndex+1] = current & 0xFF;        // 電流の下位バイト
    // 他のモーターの電流を0にする（必要に応じて調整）

    CANMessage msg(0x200, data, 8);
    if (!can.write(msg)) {
        printf("CAN send failed for Motor ID: 0x%X\n", motor_id);
    }
}

void C620::update_feedback() {
    CANMessage msg;
    while (can.read(msg)) {
        if (msg.id == (0x200 + motor_id)) {
            int16_t current_angle = (static_cast<int16_t>(msg.data[0]) << 8) | msg.data[1];
            speed = (static_cast<int16_t>(msg.data[2]) << 8) | msg.data[3];
            torque = (static_cast<int16_t>(msg.data[4]) << 8) | msg.data[5];
            temperature = msg.data[6];

            // 累積角度の更新
            int16_t delta_angle = current_angle - last_angle;
            cumulative_angle += delta_angle;
            last_angle = current_angle;
        }
    }
}

void C620::set_target_speed(int16_t target_speed) {
    this->target_speed = target_speed;
}

int16_t C620::get_speed() const {
    return speed;
}

int16_t C620::get_torque() const {
    return torque;
}

int32_t C620::get_angle() const {
    return cumulative_angle;
}

int8_t C620::get_temperature() const {
    return temperature;
}

int16_t C620::calculate_pid_output() {
    int16_t error = target_speed - speed;

    integral += error * this->ctrl_period;
    float derivative = (error - prev_error) / this->ctrl_period;

    float output = gain.kp * error + gain.ki * integral + gain.kd * derivative;

    prev_error = error;

    // 出力をトルク電流の範囲にクリップ (-16384 ～ 16384)
    if (output > 16384.0f) output = 16384.0f;
    if (output < -16384.0f) output = -16384.0f;

    return static_cast<int16_t>(output);
}

void C620::control_loop() {
    if (control_loop_active) {
        update_feedback();  // フィードバックデータを更新
        int16_t pid_output = calculate_pid_output();
        set_current(pid_output);  // 計算されたPID出力をトルク電流として設定
    }
}

void C620::stop_control_loop() {
    control_loop_active = false;
    t.detach();
    reset_pid(); // 必要に応じてPID内部変数をリセット
}

void C620::start_control_loop() {
    t.attach(callback(this, &C620::control_loop), 10ms);
    control_loop_active = true;
}

void C620::set_pid_parameters(PIDGain _gain) {
    this->gain = _gain;
}

void C620::reset_pid() {
    integral = 0.0f;
    prev_error = 0;
}

void C620::auto_tune_pid() {
    // stop_control_loop();

    // // 初期の設定
    // float ku = 0.0f; // 臨界ゲイン
    // float tu = 0.0f; // オシレーション周期
    // bool oscillating = false;
    // float current_kp = 1.0f;
    // const float increase_factor = 1.1f;

    // set_pid_parameters({current_kp, 0.0f, 0.0f});

    // // オシレーションを検出するための制御ループ
    // for (int i = 0; i < 100; ++i) {
    //     set_target_speed(1000); // 目標速度を設定
    //     start_control_loop();

    //     ThisThread::sleep_for(500ms); // 少し待つ

    //     if (!oscillating && (get_speed() > 100 || get_speed() < -100)) {
    //         // オシレーションの開始を検出
    //         oscillating = true;
    //         ku = current_kp;
    //         tu = 0.5f; // 初期値 (サンプル時間)
    //     }

    //     if (oscillating && (get_speed() < 10 && get_speed() > -10)) {
    //         // オシレーションが停止したら終わり
    //         break;
    //     }

    //     // ゲインを増加
    //     current_kp *= increase_factor;
    //     set_pid_parameters(current_kp, 0.0f, 0.0f);
    // }

    // // ゲインが設定されなかった場合の安全処理
    // if (ku == 0.0f || tu == 0.0f) {
    //     set_pid_parameters(1.0f, 0.0f, 0.0f); // デフォルトのゲイン
    // } else {
    //     // Ziegler-Nichols法に基づく最適ゲインを設定
    //     float tuned_kp = 0.6f * ku;
    //     float tuned_ki = 2.0f * tuned_kp / tu;
    //     float tuned_kd = tuned_kp * tu / 8.0f;
    //     set_pid_parameters(tuned_kp, tuned_ki, tuned_kd);
    // }

    // start_control_loop();
}

PIDGain C620::get_pid_parameters() const {
    return this->gain;
}

void C620::rotate_to_angle(int32_t target_angle, int16_t current, int16_t angle_tolerance) {
    // 制御ループを停止
    stop_control_loop();

    update_feedback(); // まず現在の角度を取得
    
    target_angle += cumulative_angle;//現在の角度から目標角度移動すると

    int32_t start_angle = cumulative_angle;
    // 現在の角度が目標角度からの許容誤差内であるかを確認
    while (abs((cumulative_angle - start_angle) - target_angle) > angle_tolerance) {
        set_current(current);  // 正方向に回転
        ThisThread::sleep_for(10ms); // 10ms待機
        update_feedback(); // 角度を再度確認
    }

    // 目標角度に達したらトルクをゼロにしてモーターを停止
    set_current(0);

    // 制御ループを再開
    start_control_loop();
}

void C620::rotate_to_angle_bySpeed(int32_t target_angle, int16_t target_speed, int16_t angle_tolerance) {
    update_feedback();
    target_angle += cumulative_angle;//現在の角度から目標角度移動すると
    int32_t start_angle = cumulative_angle;
    // 現在の角度が目標角度からの許容誤差内であるかを確認
    while (abs((cumulative_angle - start_angle) - target_angle) > angle_tolerance) {
        set_target_speed(target_speed);  // 正方向に回転
        ThisThread::sleep_for(10ms); // 10ms待機
        update_feedback(); // 角度を再度確認
    }
}