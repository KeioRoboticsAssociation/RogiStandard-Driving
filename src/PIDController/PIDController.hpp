#pragma once

typedef struct {
    float kp; //Pゲイン
    float ki; //Iゲイン
    float kd; //Dゲイン
    int frequency; //制御頻度
} PIDGain;

class PIDController{
    public:
        PIDController(); //デフォルトコンストラクタ
        PIDController(PIDGain pid_gain); //ゲインを与えて初期化
        PIDController(float kp, float ki, float kd, int frequency); //ゲインを与えて初期化

        float calculate(float error); //偏差を与えると操作量を返す．

        void reset(); //積分値をリセット

        void setFrequeny(int frequency); //周波数を変更
        int getFrequency(); //周波数を取得

        void setGain(float kp, float ki, float kd); //ゲインを変更
        void setGain(PIDGain pid_gain); //ゲインを変更

    private:
        float integral = 0.0f; //積分値
        float prevError = 0.0f; //前回の偏差
        PIDGain gain; //ゲイン
};
