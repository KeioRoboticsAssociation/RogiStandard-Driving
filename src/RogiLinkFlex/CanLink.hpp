#include <mbed.h>
#include "RogiLinkFlex/CommunicationBase.hpp"

// CanLinkクラス。CommunicationBaseを継承
class CanLink: public CommunicationBase {
    public:
        CanLink(PinName rd_pin, PinName td_pin, int frequency, EventQueue* queue, uint8_t device_id=0);
        void send(uint8_t frame_id, uint8_t* buffer, size_t size) override;
        
    private:
        void interrupt();
        void interrupt_wrapper();
        CAN can;
        EventQueue* canlink_queue;
};

// Publisher
template<typename... Args>
using CanLinkPublisher = Publisher<CanLink, Args...>;

// Subscriber
template<typename... Args>
using CanLinkSubscriber = Subscriber<CanLink, Args...>;
