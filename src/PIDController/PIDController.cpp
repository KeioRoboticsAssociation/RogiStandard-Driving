#include "PIDController.hpp"

PIDController::PIDController(): gain({0.0f, 0.0f, 0.0f, 0}){

}

PIDController::PIDController(PIDGain pid_gain): gain(pid_gain){

}

PIDController::PIDController(float kp, float ki, float kd, int frequency){
    gain.kp = kp;
    gain.ki = ki;
    gain.kd = kd;
    gain.frequency = frequency;
}

float PIDController::calculate(float error) {
    float output = gain.kp * error + gain.ki/gain.frequency * integral + gain.kd*gain.frequency * (error - prevError); //周波数を考慮
    prevError = error; //前回の偏差を更新
    integral += error; //積分値を更新

    return output; //操作量を返す
}

void PIDController::reset(){
    integral = 0.0f; //積分値をリセット
}

void PIDController::setFrequeny(int frequency){
    gain.frequency = frequency; //周波数を変更
}

int PIDController::getFrequency(){
    return gain.frequency; //周波数を取得
}

void PIDController::setGain(float kp, float ki, float kd){
    gain.kp = kp;
    gain.ki = ki;
    gain.kd = kd;
}

void PIDController::setGain(PIDGain pid_gain){
    gain = pid_gain;
}