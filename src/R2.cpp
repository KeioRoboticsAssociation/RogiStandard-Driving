#include <mbed.h>
#include <chrono>
using namespace std::chrono;

#include "R2.hpp"

enum R2_MOVEMENT{
    FORWARD_1600,
    STOP_60_1,
    BACKWARD_400,
    LEFT_650,
    FORWARD_400,
    STOP_60_2,
    BACKWARD_650,
    ROTATE_90_CW,
    BACKWARD_950
};

R2::R2(){

}

void R2::run(unsigned int movement_id){
    switch (movement_id) {
    case R2_MOVEMENT::FORWARD_1600:
        
        break;
    case R2_MOVEMENT::BACKWARD_400:
        
        break;
    default:
        printf("All motions completed!\n");
        break;
    }
}

void R2::game(){
    for(int i=0;i<=R2_MOVEMENT::BACKWARD_400;i++){
        run(i);
    }
}