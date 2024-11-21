#pragma once

#include "mbed.h"
#include "./PIDController/PIDController.hpp"


enum CtrlMode {
    CONTROL_MODE_TORQUE_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3,
};

enum C620InputMode{
    INPUT_MODE_PASSTHROUGH = 1,
    INPUT_MODE_VEL_RAMP = 2,
    INPUT_MODE_POS_FILTER = 3,
    INPUT_MODE_TRAP_TRAJ = 5,
    INPUT_MODE_TORQUE_RAMP = 6
};

typedef struct{
    uint8_t motor_id;
    int8_t direction;//1 or -1

    PIDGain gain_pos = {1.0, 0, 0, 1000};
    PIDGain gain_speed = {1.0, 0, 0, 1000};
    PIDGain gain_torque = {1.0, 0, 0, 1000};

    float max_current = 1.0;
    float max_speed = 1.0;
} RogiMaster_Config;