#include "RogiLinkFlex/UartLink.hpp"

constexpr uint8_t ROGICTLS_FRAME_ID = 0x16;

class EspController {  
    public:
        enum {
            BUTTON_A = 1,
            BUTTON_B = 2,
            BUTTON_X = 4,
            BUTTON_Y = 8,
            BUTTON_SHOULDER_L = 16,
            BUTTON_SHOULDER_R = 32,
            BUTTON_TRIGGER_L = 64,
            BUTTON_TRIGGER_R = 128,
            BUTTON_THUMB_L = 256,
            BUTTON_THUMB_R = 512,
        };

        enum {
            MISC_BUTTON_SYSTEM = 1,    // AKA: PS, Xbox, etc.
            MISC_BUTTON_SELECT = 2,    // AKA: Select, Share, Create, -
            MISC_BUTTON_START = 4,      // AKA: Start, Options, +
            MISC_BUTTON_CAPTURE = 8,  // AKA: Mute, Capture, Share

            // Deprecated
            MISC_BUTTON_BACK = MISC_BUTTON_SELECT,
            MISC_BUTTON_HOME = MISC_BUTTON_START,
        };

        typedef struct {
            uint8_t controller_id;
            int32_t axisX;
            int32_t axisY;
            int32_t axisRX;
            int32_t axisRY;
            uint16_t buttons;
            uint16_t misc_buttons; 
        } data_t;


        EspController(PinName tx, PinName rx) 
          : uart_link(tx, rx, 115200), uart_link_subscriber(uart_link, ROGICTLS_FRAME_ID)
        {
            uart_link_subscriber.set_callback(callback(this, &EspController::onReceive));
        }

        // Axis
        int32_t axisX() const { return data.axisX; }
        int32_t axisY() const { return data.axisY; }
        int32_t axisRX() const { return data.axisRX; }
        int32_t axisRY() const { return data.axisRY; }


        //
        // Shared between Mouse & Gamepad
        //

        // Returns the state of all buttons.
        uint16_t buttons() const { return data.buttons; }

        // Returns the state of all misc buttons.
        uint16_t miscButtons() const { return data.misc_buttons; }

        // To test one button at a time.
        bool a() const { return buttons() & BUTTON_A; }
        bool b() const { return buttons() & BUTTON_B; }
        bool x() const { return buttons() & BUTTON_X; }
        bool y() const { return buttons() & BUTTON_Y; }
        bool l1() const { return buttons() & BUTTON_SHOULDER_L; }
        bool l2() const { return buttons() & BUTTON_TRIGGER_L; }
        bool r1() const { return buttons() & BUTTON_SHOULDER_R; }
        bool r2() const { return buttons() & BUTTON_TRIGGER_R; }
        bool thumbL() const { return buttons() & BUTTON_THUMB_L; }
        bool thumbR() const { return buttons() & BUTTON_THUMB_R; }

        // Misc buttons
        bool miscSystem() const { return miscButtons() & MISC_BUTTON_SYSTEM; }
        bool miscSelect() const { return miscButtons() & MISC_BUTTON_SELECT; }
        bool miscStart() const { return miscButtons() & MISC_BUTTON_START; }
        bool miscCapture() const { return miscButtons() & MISC_BUTTON_CAPTURE; }

        // Deprecated
        bool miscBack() const { return miscSelect(); }
        bool miscHome() const { return miscStart(); }


    private:
        void onReceive(
            uint8_t controller_id,
            int32_t axisX,
            int32_t axisY,
            int32_t axisRX,
            int32_t axisRY,
            uint16_t buttons,
            uint16_t misc_buttons
        ) {
            data.axisX = axisX;
            data.axisY = axisY;
            data.axisRX = axisRX;
            data.axisRY = axisRY;
            data.buttons = buttons;
            data.misc_buttons = misc_buttons;
        }

        UartLink uart_link;

        UartLinkSubscriber<
            uint8_t,
            int32_t,
            int32_t,
            int32_t,
            int32_t,
            uint16_t,
            uint16_t
        > uart_link_subscriber;

        data_t data;
};