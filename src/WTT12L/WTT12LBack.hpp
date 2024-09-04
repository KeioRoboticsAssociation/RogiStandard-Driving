#include <mbed.h>

class WTT12LBack {
    public:
        WTT12LBack(PinName pinA);
        int getOutput(); // ターゲット距離

        DigitalIn rangefinder;
    private:
         
};