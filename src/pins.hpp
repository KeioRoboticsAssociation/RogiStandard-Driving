#pragma once
#include <PinNames.h>
#include "settings.hpp"

namespace DigitalOutPins
{
    constexpr PinName MOTOR1_DIR = PA_9; // モータ1の方向
    constexpr PinName MOTOR2_DIR = PB_2; // モータ2の方向
    constexpr PinName MOTOR3_DIR = PB_1; // モータ3の方向
}

namespace PwmOutPins
{
    constexpr PinName MOTOR1_PWM = PB_13; // モータ1のPWM
    constexpr PinName MOTOR2_PWM = PB_14; // モータ2のPWM
    constexpr PinName MOTOR3_PWM = PB_15; // モータ3のPWM
}

namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B = PC_12; // 測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B = PB_7; // 測定輪エンコーダ2のB相
    constexpr PinName MEASURING_ENCODER3_B = PC_3; // 測定輪エンコーダ3のB相
}

namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A = PA_15; // 測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A = PC_2; // 測定輪エンコーダ2のA相
    constexpr PinName MEASURING_ENCODER3_A = PC_0; // 測定輪エンコーダ3のA相
}



namespace AnalogInPins
{
    #if USE_LASER_WTT12L
    constexpr PinName RIGHT_WTT12L = PA_7;
    constexpr PinName BACK_WTT12L = PA_8;
    #endif
}

namespace AnalogOutPins
{
    
}

namespace ConsolePins
{
    #if USE_GYROSENSOR_BNO055
    constexpr PinName CONSOLE_TX = PB_1;
    constexpr PinName CONSOLE_RX = PB_2;
    #endif
}

namespace UartPins
{
    
}

namespace I2cPins
{
    
}