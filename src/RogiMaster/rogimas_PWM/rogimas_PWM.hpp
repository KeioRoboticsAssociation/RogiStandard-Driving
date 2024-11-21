#pragma once

#include <mbed.h>

namespace C620{

constexpr float C620_MAX_SPEED = (400.0f / 60.0f); // 400rpm

class C620_PWM{
public:
    C620_PWM(PwmOut& _pwm, bool _direction, InterruptIn& _caliburation_button, bool _pulluped = true);
    ~C620_PWM();

    void setVel(float _vel);
    float getVel(){
        return vel;
    }

    /**
     * @details コンストラクタで呼ばれます。
     *          ボタンの入力がされるまで、2msのONパルスが出力されます。
     *          ボタンの入力後1msのONパルスが出よくされます。その後ボタンを押すとキャリ部完了します。
     *          ともにC620のLEDが緑色に光ったら押してください。
     */
    void calibration();

    bool is_calibrated(){
        return is_calibrated_;
    }

private:
    PwmOut& pwm;
    bool direction;
    InterruptIn& button;

    bool is_calibrated_;

    bool pulluped;

    bool button1 = false;
    bool button2 = false;

    Ticker t;

    float vel = 0;
    void ctrl_callback();
}; // class C620_PWM

}; // namespace C620