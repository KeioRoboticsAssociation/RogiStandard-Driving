#pragma once

#include "mbed.h"
#include "./PIDController/PIDController.hpp"
#include "./rogimas_base/rogimas_base.hpp"

class RogiMaster_axis{
public:
    RogiMaster_axis(RogiMaster_Config _config); // cppで書く

    void idle(){
        is_closed_loop_ = false;
    } // 電流入力を0にする
    void closed_loop(){
        is_closed_loop_ = true;
    }
    void set_CtrlMode(int _mode){
        ctrlmode_ = _mode;
        if (_mode == CONTROL_MODE_POSITION_CONTROL)
        {
            pid_position_.reset();
        } 
    }
    void operator=(float _input_passthrough);//cppで書く

    // feedbackで得られる、現在の状態
    float angle_rad_;
    float speed_rad_;
    float current_;

    int16_t input_current_;
    int16_t calc_current();//cppで書く
    
private:
    uint8_t motor_id_;
public:
    int8_t direction_;
private:
    uint8_t ctrlmode_;

    bool is_closed_loop_;

    PIDController pid_position_;
    PIDController pid_speed_;
    PIDController pid_torque_;

public:
    float max_current_;
    float max_speed_;

private:
    float target_torque_; // Nm
    float target_speed_; // rot/s
    float target_rotation_; // rot, 2pi rad = 1 rotation

    float last_angle_rad_;
    int rot_count_;

};// class C620_axis