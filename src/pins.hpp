#pragma once
#include <PinNames.h>

namespace DigitalOutPins
{
    constexpr PinName MOTOR1_DIR = PA_4; // モータ1の方向
    constexpr PinName MOTOR2_DIR = PC_14; // モータ2の方向
    constexpr PinName MOTOR3_DIR = PA_7; // モータ3の方向
}

namespace PwmOutPins
{
    constexpr PinName MOTOR1_PWM = PA_11; // モータ1のPWM
    constexpr PinName MOTOR2_PWM = PA_9; // モータ2のPWM
    constexpr PinName MOTOR3_PWM = PA_10; // モータ3のPWM
}

namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B = PC_0; // 測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B = PC_1; // 測定輪エンコーダ2のB相
    constexpr PinName MEASURING_ENCODER3_B = PB_2; // 測定輪エンコーダ3のB相
}

namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A = PB_1; // 測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A = PA_12; // 測定輪エンコーダ2のA相
    constexpr PinName MEASURING_ENCODER3_A = PB_1; // 測定輪エンコーダ3のA相
}



namespace AnalogInPins
{
    
}

namespace AnalogOutPins
{
    
}

namespace UartPins
{
    
}

namespace I2cPins
{
    
}
