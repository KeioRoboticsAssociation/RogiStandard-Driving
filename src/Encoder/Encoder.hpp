#pragma once
#include <mbed.h>

class Encoder {
    public:
        Encoder(PinName interrupt_in_pin, PinName digital_in_pin, int resolution=2048, bool direction=0, bool dual=false);

        int getCount();
        float getRadians();
        float getDegrees();
        float getRotations();
        float countToRadians(int count);
        float countToDegrees(int count);
        float countToRotations(int count);
        
    private:
        DigitalIn digital_in;
        InterruptIn interrupt_in;

        int converted_resolution; // 1回転あたりのカウント数
        int count;

        // 割り込みハンドラ
        void interrupt(int sgn);
};