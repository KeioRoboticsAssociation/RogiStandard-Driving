#pragma once

#include <array>
#include "mbed.h"
#include "./PIDController/PIDController.hpp"

#include "./rogimas_CANoriginal/rogimas_axis.hpp"
#include "./rogimas_base/rogimas_base.hpp"

template <int N>
class RogiMaster{
private:
    CAN &can_;

    float input_current[N]; // 全軸の電流を格納。受信時には１軸分のデータを変更するが、送信は全軸行うので。
    float last_angle[N];

    // connection check
    Timer timer;
    Ticker ticker;
    bool is_connected_;

    void calc_all_current_data(uint8_t* _data){
        for(int i = 0; i < N; i++){
            float current = axis[i].calc_current();
            _data[2 * i] = (int16_t)(current * std::pow(2, 16)) >> 8;
            _data[2 * i + 1] = (int16_t)(current * std::pow(2, 16));
        }

    }

    void can_callback(){
        CANMessage msg;
        if (can_.read(msg))
        {
            uint8_t motor_id = msg.id - 0x200;
            if (motor_id > N)
            {
                return;
            }
            
            
            // process feedback data
            float current_angle_rad = axis[motor_id].direction * (msg.data[0] << 8 | msg.data[1]) / std::pow(2, 13) * 2 * M_PI; // angle resolution is 13bit
            
            if (this->last_angle[motor_id] > current_angle_rad)
            {
                this->axis[motor_id].rot_count++;
            }
            else if (this->angle[motor_id] < current_angle_rad)
            {
                this->axis[motor_id].rot_count--;
            }
            
            current_angle_rad += 2 * M_PI * this->axis[motor_id].rot_count;

            this->axis->angle_rad[motor_id] = current_angle_rad;
            this->axis->speed_rad[motor_id] = axis[motor_id].direction * (msg.data[2] << 8 | msg.data[3]) * 2 * M_PI / 60; // rpm -> rad/s
            this->axis->current[motor_id] = axis[motor_id].direction * (msg.data[4] << 8 | msg.data[5]) / std::pow(2, 15); // torque resolution is 15bit?

            // process control data
            axis[motor_id].input_current = axis[motor_id].calc_current();
            // ↑は、↓内部でaxis[n].input_currentを参照してるので、先に代入しておく必要がある
            int8_t data[16];
            calc_all_current_data(data);

            // send control data
            CANMessage msg1(0x200, data, 8);
            CANMessage msg2(0x1FF, &data[8], 8);
            can_.write(msg1);
            can_.write(msg2);
            
            timer.reset();
            timer.start();
        }
        
    }

public:
    RogiMaster(CAN &_can, std::array<RogiMaster_Config, N> _config)
        : can_(_can)
        , axis(_config)
    {
        can_.frequency(1000000);
        can_.reset();
        can_.attach(callback(this, &RogiMaster::can_callback), CAN::RxIrq);

        ticker.attach([this]{
            if(timer.read_ms() > 100){
                idle();
                this->ticker.detach();
                this->timer.stop();

                this->is_connected_ = false;
            }
        }, std::chrono::milliseconds(100));
    }
    
    RogiMaster_axis axis[N];
    
    bool is_connected(){
        return is_connected_;
    }
};
