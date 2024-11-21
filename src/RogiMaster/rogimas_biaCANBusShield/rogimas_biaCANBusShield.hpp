/*
このライブラリはODriveライクな感じにしてます。

default pin assign is CS_A = D9, INIT_A = D3

*/

#pragma once

#include <mbed.h>
#include "C620.hpp"

class C620_biaCANBusShield{
public:
    // @param direction true = CCW
    C620_biaCANBusShield(
        SPI& _spi,
        DigitalOut& _cs,
        uint8_t _motor_id,
        bool _direction,
        PIDGain _currentGain,
        PIDGain _velGain,
        PIDGain _posGain
    );
    
    ~C620_biaCANBusShield();

    // @param _current -20.0A ~ 20.0A. 大きすぎたら「デカすぎ」orintfして何もしない
    void setCurrent(float _current);
    void setSpeed(float _anglVel);
    void setPos(float _pos);

    float getCurrent() const;
    float getSpeed() const;
    float getPos() const;
    float getTemperature() const;

    void start_torque_ctrl();
    void stop_torque_ctrl();

    void start_velocity_ctrl();
    void stop_velocity_ctrl();

    void start_ramped_velocity_ctrl();
    void stop_ramped_velocity_ctrl();

    void start_pos_ctrl();
    void stop_pos_ctrl();

    void start_trajectry_pos_ctrl();
    void stop_trajectry_pos_ctrl();

private:
    void current_callback();
    void velocity_callback();
    void pos_callback();

    Ticker t_current;
    Ticker t_vel;
    Ticker t_pos;

    SPI& spi;
    DigitalOut& cs;
    MotorStatus motor_status;

    PIDGain current_gain;
    PIDGain vel_gain;
    PIDGain pos_gain;

    float current_period;
    float vel_period;
    float pos_period;
};