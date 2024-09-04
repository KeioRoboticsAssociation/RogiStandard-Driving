#include <mbed.h>

class WTT12L {
    public:
        WTT12L(PinName pinA, PinName pinB);
        int getOutput1(); // ターゲット距離より近い
        int getOutput2(); // ターゲット距離

        DigitalIn rangefinder1;
        DigitalIn rangefinder2;
    private:
         
};