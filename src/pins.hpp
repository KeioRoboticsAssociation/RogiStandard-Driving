// #pragma once
// #include <PinNames.h>

// namespace DigitalOutPins
// {
//     constexpr PinName MOTOR1_DIR = PA_4; // モータ1の方向
//     constexpr PinName MOTOR2_DIR = PC_14; // モータ2の方向
//     constexpr PinName MOTOR3_DIR = PA_7; // モータ3の方向
// }

// namespace PwmOutPins
// {
//     constexpr PinName MOTOR1_PWM = PA_11; // モータ1のPWM
//     constexpr PinName MOTOR2_PWM = PA_9; // モータ2のPWM
//     constexpr PinName MOTOR3_PWM = PA_10; // モータ3のPWM
// }

// namespace DigitalInPins
// {
//     constexpr PinName MEASURING_ENCODER1_B = PC_0; // 測定輪エンコーダ1のB相
//     constexpr PinName MEASURING_ENCODER2_B = PC_1; // 測定輪エンコーダ2のB相
//     constexpr PinName MEASURING_ENCODER3_B = PB_2; // 測定輪エンコーダ3のB相
// }

// namespace InterruptInPins
// {
//     constexpr PinName MEASURING_ENCODER1_A = PB_1; // 測定輪エンコーダ1のA相
//     constexpr PinName MEASURING_ENCODER2_A = PA_12; // 測定輪エンコーダ2のA相
//     constexpr PinName MEASURING_ENCODER3_A = PB_1; // 測定輪エンコーダ3のA相
// }



// namespace AnalogInPins
// {
    
// }

// namespace AnalogOutPins
// {
    
// }

// namespace UartPins
// {
    
// }

// namespace I2cPins
// {
    
// }

#pragma once

#include <PinNames.h>

namespace DigitalOutPin
{
    constexpr PinName DIR_MECHA_R = PA_4; // md1
    constexpr PinName DIR_MECHA_L = PB_7; // md2 kawatta
    constexpr PinName DIR_OMNI = PB_5; // md3 kawatta
}

namespace DigitalInPin
{
    
}

namespace PwmOutPin
{
    // Timer1
    constexpr PinName PWM_MECHA_R = PA_11; // md1
    constexpr PinName PWM_MECHA_L = PA_10; // md2
    constexpr PinName PWM_OMNI = PA_9; // md3

    // Timer3
    constexpr PinName PWM_ESC = PB_6;

    constexpr PinName PWM_TANK_R = PC_9; //servo1
    constexpr PinName PWM_TANK_C = PC_8; //servo2
    constexpr PinName PWM_TANK_L = PA_6; //servo3
    
    constexpr PinName PWM_SERVO_BASE = PB_2; //servo4
    constexpr PinName PWM_SERVO_TIP = PB_8; //servo5
    constexpr PinName PWM_SERVO_BLDC = PB_9;    //servo6
} 

namespace InterruptInPin
{
    //enc 1
    constexpr PinName ENC_MECHA_R_A = PB_1;
    constexpr PinName ENC_MECHA_R_B = PC_0;

    //enc 2
    constexpr PinName ENC_MECHA_L_A = PA_12;
    constexpr PinName ENC_MECHA_L_B = PC_1;

    //enc 3
    constexpr PinName ENC_OMNI_A = PB_13;
    constexpr PinName ENC_OMNI_B = PC_11;

    //enc 4
    constexpr PinName OdomR_A = PB_14;
    constexpr PinName OdomR_B = PC_10;
    
    //enc5
    constexpr PinName OdomL_A = PB_15;
    constexpr PinName OdomL_B = PC_7;
}

namespace UARTPin{
    constexpr PinName UART_TX = PA_1;
    constexpr PinName UART_RX = PA_0;
}

namespace I2CPin{
    constexpr PinName I2C2_SDA = PC_12;
    constexpr PinName I2C2_SCL = PB_10;

    constexpr PinName I2C3_SDA = PB_4;
    constexpr PinName I2C3_SCL = PB_10;
}

namespace AnalogInPin {
    constexpr PinName TOF = PB_0;
}