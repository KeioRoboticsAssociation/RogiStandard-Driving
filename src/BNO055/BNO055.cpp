#include "BNO055.hpp"
#include <math.h>

BNO055::BNO055(PinName tx, PinName rx, std::chrono::milliseconds internal): serial(tx, rx)
{
    serial.baud(115200);
    
    serial.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bit */ 1
    );

    writeByte(0x3D, 0x0C);

    wait_us(20000);
    
    ticker.attach([this]{requestValue();}, internal);

    timer.start();

    while(!_active){
        wait_ns(1);
        if (timer.elapsed_time() > CONNECTION_TIMEOUT){
            printf("BNO055 connection timeout.\n");
            break;
        }
    }
    reset();
}

void BNO055::writeByte(unsigned char reg_add, unsigned char value){
    char data[] = {START_BYTE, WRITE_BYTE, reg_add, 0x01, value};
    serial.write(data, 5);
}

void BNO055::requestValue(){
    char data[] = {START_BYTE, READ_BYTE, TARGET_REGISTER, 0x02};
    serial.write(data, 4);
    serial.attach([this]{interrupt();});
}

void BNO055::interrupt(){
    char c;
    serial.read(&c, 1);

    switch (receive_process)
    {
    case STARTING:
        if(c == RESPONSE_BYTE){
            receive_process = SET_LENGTH;
        }
        break;
    
    case SET_LENGTH:
        receive_length = c;
        receive_index = 0;
        receive_process = RECEIVING;
        break;

    case RECEIVING:
        receive_buffer[receive_index] = c;
        receive_index++;

        if(receive_index == receive_length){
            receive_process = STARTING;
            if (receive_length == 2){
                updateValue();
            }
        }
        break;
    
    default:
        break;
    }
}


void BNO055::updateValue(){
    int new_value = (int16_t(receive_buffer[1]) << 8) | int16_t(receive_buffer[0]);

    if(raw_value > 3 * RESOLUTION / 4 && new_value < RESOLUTION / 4){
        zero_point -= RESOLUTION;
    }else if(raw_value < RESOLUTION / 4 && new_value > 3 * RESOLUTION / 4){
        zero_point += RESOLUTION;
    }

    raw_value = new_value;
    _active = true;
}

void BNO055::reset(int value){
    zero_point = raw_value - value;
}


int BNO055::getValue(){
    return raw_value - zero_point;
}

float BNO055::getDegrees(){
    return - 360.0f * getValue() / RESOLUTION;
}

float BNO055::getRadians(){
    return - 2 * M_PI * getValue() / RESOLUTION;
}


void BNO055::setRadians(float radians){
    this->reset(- radians * RESOLUTION / (2 * M_PI));
}

bool BNO055::active(){
    return _active;
}


