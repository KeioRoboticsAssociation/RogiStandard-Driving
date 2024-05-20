#include "RogiLinkFlex/CanLink.hpp"

// コンストラクタ
CanLink::CanLink(PinName rd_pin, PinName td_pin, int frequency, EventQueue* queue, uint8_t device_id)
 : CommunicationBase(nullptr, device_id), can(rd_pin, td_pin), canlink_queue(queue) // コンストラクタでCANクラスを初期化する。イベントキューはCanLink側で管理するためnullptrを渡す
{
    can.frequency(frequency);
    can.attach(callback(this, &CanLink::interrupt_wrapper), CAN::RxIrq); // 受信割り込みを登録
}

// 送信処理
void CanLink::send(uint8_t frame_id, uint8_t* buffer, size_t size)
{
    CANMessage msg;
    msg.id = frame_id;
    msg.len = size;
    memcpy(msg.data, buffer, size);
    can.write(msg);
}

// 受信割り込み
void CanLink::interrupt()
{
    CANMessage msg;
    while (can.read(msg)) {
        on_receive(msg.id, msg.data, msg.len);
    }
}

void CanLink::interrupt_wrapper()
{
    if (canlink_queue != nullptr){
        canlink_queue->call(callback(this, &CanLink::interrupt));
    }else{
        interrupt();
    }
}