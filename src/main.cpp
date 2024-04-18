#if 0
#include <mbed.h>
#include "Encoder/Encoder.hpp"
#include "OdomWheel/Odometry.hpp"

Encoder encoder1(D2, D3, 2048, 0, false);
Encoder encoder2(D4, D5, 2048, 0, false);
Encoder encoder3(D6, D7, 2048, 0, false);

std::array<WheelConfig, 3> config = {
    WheelConfig{100.0, 100.0, 0.0, M_PI / 4}, 
    WheelConfig{100.0, -100.0, 0.0, 3 * M_PI / 4}, 
    WheelConfig{100.0, 0.0, 100.0, 0.0}};

WheelOdometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});


int main() {
    
}
#endif

#if 1

#include <mbed.h>
#include "OdomWheel/Odometry.hpp"

Encoder encoder1(D2, D3, 2048, 0, false);
Encoder encoder2(D4, D5, 2048, 0, false);

std::array<WheelConfig, 2> config = {
    WheelConfig{100.0, 100.0, 0.0, M_PI / 4}, 
    WheelConfig{100.0, -100.0, 0.0, 2 * M_PI / 4}
};

TwoWheelOdometry odometry(config, {&encoder1, &encoder2});

int main() {
    
}




#endif


