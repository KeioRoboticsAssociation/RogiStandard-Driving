#include <mbed.h>
#include "RogiLinkFlex/CommunicationBase.hpp"

// UartLinkクラス。CobsEncodedCommunicationBaseを継承
class UartLink : public CobsEncodedCommunicationBase {
    public:
        UartLink(PinName tx, PinName rx, int baudrate, EventQueue* queue=nullptr, uint8_t device_id=0);

        // 送信処理
        void send_raw(uint8_t* data, uint8_t size) override;

    private:
        // 受信割り込み
        void interrupt();

        UnbufferedSerial uart;
        uint8_t receive_buffer[BUFFER_SIZE];
        uint8_t receive_buffer_index = 0;
};

// Publisher
template<typename... Args>
using UartLinkPublisher = Publisher<UartLink, Args...>;

// Subscriber
template<typename... Args>
using UartLinkSubscriber = Subscriber<UartLink, Args...>;