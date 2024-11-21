#include "rogimas_axis.hpp"

RogiMaster_axis::RogiMaster_axis(RogiMaster_Config _config)
    : motor_id_(_config.motor_id),
    direction_(_config.direction),
    pid_position_(_config.gain_pos),
    pid_speed_(_config.gain_speed),
    pid_torque_(_config.gain_torque),
    max_current_(_config.max_current),
    max_speed_(_config.max_speed)
{
    is_closed_loop_ = false;
    ctrlmode_ = CONTROL_MODE_VELOCITY_CONTROL;
}

int16_t RogiMaster_axis::calc_current(){
    if(!is_closed_loop_){
        return 0;
    }

    int16_t u;
    if(ctrlmode_ == CONTROL_MODE_TORQUE_CONTROL){
        float torque = pid_torque_.calculate(target_torque_ - current_);
        u = (int16_t)(torque * std::pow(2, 16));
    }
    else if(ctrlmode_ == CONTROL_MODE_VELOCITY_CONTROL){
        float vel = pid_torque_.calculate(target_speed_ - speed_rad_);
        u = (int16_t)(vel * std::pow(2, 16));
    }
    else if(ctrlmode_ == CONTROL_MODE_POSITION_CONTROL){
        float pos = pid_torque_.calculate(target_speed_ - speed_rad_);
        u = (int16_t)(pos * std::pow(2, 13));
    }
    else{
        u = 0;
    }

    return u;
}

void RogiMaster_axis::operator=(float _input_passthrough){
    if(ctrlmode_ == CONTROL_MODE_TORQUE_CONTROL){
        target_torque_ = _input_passthrough;
    }
    else if(ctrlmode_ == CONTROL_MODE_VELOCITY_CONTROL){
        target_speed_ = _input_passthrough;
    }
    else if(ctrlmode_ == CONTROL_MODE_POSITION_CONTROL){
        target_rotation_ = _input_passthrough;
    }
}


