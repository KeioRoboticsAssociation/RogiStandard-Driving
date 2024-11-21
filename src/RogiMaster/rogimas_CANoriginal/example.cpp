#if 0
#include "C620_CANoriginal/C620_kai.hpp"

CAN can(PB_8, PB_9, 1000000);
C620<1> c620(can);

int main(){
    c620.axis[0].set_ctrlmode(C620_CTRL_MODE_POSITION);

    c620.axis[0] = 0.0f;
    wait_us(2 * std::pow(10, 6));

    c620.axis[0] = 1.0f;
    wait_us(2 * std::pow(10, 6));

    c620.axis[0].set_ctrlmode(C620_CTRL_MODE_VELOCITY);

    c620.axis[0] = 0.0f;
    wait_us(2 * std::pow(10, 6));

    c620.axis[0] = 1.0f;
    wait_us(2 * std::pow(10, 6));

    c620.axis[0].set_ctrlmode(C620_CTRL_MODE_TORQUE);

    c620.axis[0].target_torque = 1.0f;
    wait_us(2 * std::pow(10, 6));

    c620.axis[0].target_torque = 0.0f;
    wait_us(2 * std::pow(10, 6));
}

#endif