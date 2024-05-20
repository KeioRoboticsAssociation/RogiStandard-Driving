#include "RogiLinkFlex/UartLink.hpp"

// コンストラクタ
UartLink::UartLink(PinName tx, PinName rx, int baudrate, EventQueue* queue, uint8_t device_id)
 : CobsEncodedCommunicationBase(queue, device_id), uart(tx, rx, baudrate)
{
    uart.baud(baudrate);
    uart.attach(callback(this, &UartLink::interrupt), UnbufferedSerial::RxIrq); // 受信割り込みを登録
}

// 送信処理
void UartLink::send_raw(uint8_t* data, uint8_t size)
{
    //printf("send\n");
    uart.write(data, size);
}

// 受信割り込み
void UartLink::interrupt()
{
    if (!uart.readable()) {
        return;
    }

    uint8_t c;
    uart.read(&c, 1); // 1バイト読み込み

    // 受信バッファに格納
    receive_buffer[receive_buffer_index] = c;
    receive_buffer_index++;

    if (receive_buffer_index >= BUFFER_SIZE - 1) {
        // バッファがオーバーフローしたらリセット
        receive_buffer_index = 0;
        return;
    }

    if (c == 0x00) {
        uint8_t copied[BUFFER_SIZE]; // 受信バッファのコピー
        memcpy(copied, receive_buffer, receive_buffer_index); // 受信バッファをコピー
        on_receive_raw(copied, receive_buffer_index); // 受信処理
        receive_buffer_index = 0; // 受信バッファをリセット
    }
}