#pragma once
#include <PinNames.h>
#include "settings.hpp"

namespace DigitalOutPins
{
    constexpr PinName MOTOR1_DIR = PA_9; // モータ1の方向
    constexpr PinName MOTOR2_DIR = PB_2; // モータ2の方向
    constexpr PinName MOTOR3_DIR = PB_1; // モータ3の方向
    constexpr PinName MOTOR4_DIR = PA_12; // モータ4の方向
    #if USE_PROPELLER
    constexpr PinName PROPELLER_MOTOR_DIR = PA_6;
    #endif
    #if USE_AIRCYLINDER
    constexpr PinName CYLINDER1 = PA_5;
    #endif
}

namespace PwmOutPins
{
    constexpr PinName MOTOR1_PWM = PB_13; // モータ1のPWM
    constexpr PinName MOTOR2_PWM = PB_14; // モータ2のPWM
    constexpr PinName MOTOR3_PWM = PB_15; // モータ3のPWM
    constexpr PinName MOTOR4_PWM = PA_11; // モータ4のPWM
    #if USE_PROPELLER
    constexpr PinName PROPELLER_MOTOR_PWM = PB_9;
    #endif
}

namespace DigitalInPins
{
    constexpr PinName ENCODER_FL_B = PC_12; // エンコーダ1のB相
    constexpr PinName ENCODER_FR_B = PB_7; // エンコーダ2のB相
    constexpr PinName ENCODER_BL_B = PC_3; // エンコーダ3のB相
    constexpr PinName ENCODER_BR_B = PA_4; // エンコーダ4のB相
    constexpr PinName MEASURING_ENCODER1_B = PA_0; // 測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B = PB_4; // 測定輪エンコーダ2のB相
    #if PERFORMANCE_ROBOT
    constexpr PinName START_SWITCH = PC_6;
    #endif
    #if USE_LASER_WTT12L
    constexpr PinName RIGHT_WTT12L_1 = PB_6;
    constexpr PinName RIGHT_WTT12L_2 = PA_7;
    constexpr PinName BACK_WTT12L = PB_8;
    #endif
}

namespace InterruptInPins
{
    constexpr PinName ENCODER_FL_A = PA_15; // エンコーダ1のA相
    constexpr PinName ENCODER_FR_A = PC_2; // エンコーダ2のA相
    constexpr PinName ENCODER_BL_A = PA_13; // エンコーダ3のA相
    constexpr PinName ENCODER_BR_A = PB_0; // エンコーダ4のA相右前
    constexpr PinName MEASURING_ENCODER1_A = PA_1; // 測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A = PB_5; // 測定輪エンコーダ2のA相
}



namespace AnalogInPins
{
}

namespace AnalogOutPins
{
    
}

namespace UartPins
{
    #if USE_GYROSENSOR_BNO055
    constexpr PinName CONSOLE_TX = PC_10;
    constexpr PinName CONSOLE_RX = PC_11;
    #endif
}

namespace I2cPins
{
    
}