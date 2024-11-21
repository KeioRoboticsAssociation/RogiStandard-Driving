#include "C620_biaCANBusShield.hpp"

C620_biaCANBusShield::C620_biaCANBusShield( 
        SPI& _spi,
        DigitalOut& _cs,
        uint8_t _motor_id,
        bool _direction,
        PIDGain _currentGain,
        PIDGain _velGain,
        PIDGain _posGain)
    : spi(_spi)
    , cs(_cs)
    , motor_status({_motor_id, _direction, 0, 0, 0})
    , current_gain(_currentGain)
    , vel_gain(_velGain)
    , pos_gain(_posGain)
{
    // communication init

    cs.write(1); // deselect devive on CAN Bus Shield

    spi.format(8, 3); // 第2引数、MCP2515が載ってるので0or3
    spi.frequency(1 * 1000 * 1000);

    cs.write(0); // select the device

    // ctrl init
    current_period = 1.0f / current_gain.frequency;
    vel_period = 1.0f / vel_gain.frequency;
    pos_period = 1.0f / pos_gain.frequency;

    t_current.attach(callback(this, &C620_biaCANBusShield::current_callback), current_period);
    t_vel.attach(callback(this, &C620_biaCANBusShield::current_callback), vel_period);
    t_pos.attach(callback(this, &C620_biaCANBusShield::current_callback), pos_period);
}

C620_biaCANBusShield::~C620_biaCANBusShield(){
    t_current.detach();
    t_vel.detach();
    t_pos.detach();
}

void C620_biaCANBusShield::setCurrent(float _current){
    if(abs(_current) > 20.0f){
        printf("入力デカすぎ");
        return ;
    }

    int16_t current_data = (int16_t)(_current / 20.0f * powf(2, 15));


}