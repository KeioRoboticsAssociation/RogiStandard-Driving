#include <mbed.h>

class WTT12L {
    public:
        WTT12L(PinName pinA);
        bool whetherclose(float target_threshold);

        AnalogIn rangefinder;
    private:
         
};