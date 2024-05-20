#include "RogiLinkFlex/CommunicationBase.hpp"

namespace SystemMessages {
    const uint8_t SYSTEM_MESSAGE_FRAME_ID = 0x00;

    enum SystemMessageEnum {
        CONNECTION_CHECK = 1, // 接続確認
        CONNECTION_CHECK_REPLY = 2, // 接続確認の返信 (デバイスIDを送信)
        ACK = 3, // ACK
        ERROR = 4, // エラー
    };
}

// コンストラクタ
CommunicationBase::CommunicationBase(EventQueue* queue, uint8_t device_id) : queue(queue)
{
    // デバイスIDを設定
    this->device_id = device_id;

    // システムメッセージの受信コールバックを登録
    add_callback(SystemMessages::SYSTEM_MESSAGE_FRAME_ID, [this](uint8_t* buffer, size_t size) {
        this->on_received_system_message(buffer, size);
    });
}

// 送信処理(継承先で実装する)
void CommunicationBase::send(uint8_t frame_id, uint8_t* data, size_t size) {
    // 実装は省略
    
}

// コールバック登録
void CommunicationBase::add_callback(uint8_t frame_id, std::function<void(uint8_t*, size_t)> callback) {
    if (callback_map.find(frame_id) == callback_map.end()) {
        callback_map[frame_id] = std::vector<std::function<void(uint8_t*, size_t)>>();
    }
    callback_map[frame_id].push_back(callback);
}

// 受信時の処理
void CommunicationBase::on_receive(uint8_t frame_id, uint8_t* buffer, size_t size) {
    if (callback_map.find(frame_id) == callback_map.end()) {
        return;
    }
    for (auto &callback : callback_map[frame_id]) {
        if (queue != nullptr){
            queue->call(callback, buffer, size);
        }else{
            callback(buffer, size);
        }
    }
}

// 受信時の処理(システムメッセージ)
void CommunicationBase::on_received_system_message(uint8_t* buffer, size_t size) {
    uint8_t send_buffer[BUFFER_SIZE];

    if (size == 0) {
        return;
    }

    switch (buffer[0]) {
        case SystemMessages::CONNECTION_CHECK: // 接続確認・デバイスIDの送信
            send_buffer[0] = SystemMessages::CONNECTION_CHECK_REPLY;
            send_buffer[1] = device_id;
            send(SystemMessages::SYSTEM_MESSAGE_FRAME_ID, send_buffer, 2);
            break;

        case SystemMessages::ACK: // ACK
            break;
        
        default:
            break;
    }
}


constexpr uint8_t HEADER_LENGTH = 3;

// コンストラクタ
CobsEncodedCommunicationBase::CobsEncodedCommunicationBase(EventQueue* queue, uint8_t device_id)
 : CommunicationBase(queue, device_id) 
{
    
}

void CobsEncodedCommunicationBase::send_raw(uint8_t* data, uint8_t size) {
    // 実装は省略
}

void CobsEncodedCommunicationBase::encode_cobs(uint8_t *dst, uint8_t *src, uint8_t len) {
    int last_zero_index = 0;

    for (int i = 1; i < len + 1; i++) {
        if (src[i - 1] == 0) {
            dst[last_zero_index] = i - last_zero_index;
            dst[i] = 0;
            last_zero_index = i;
        } else {
            dst[i] = src[i - 1];
        }
    }
    
    dst[len + 1] = 0;
    dst[last_zero_index] = len - last_zero_index + 1;

    return;
}


void CobsEncodedCommunicationBase::decode_cobs(uint8_t *dst, uint8_t *src, uint8_t len) {
    int next_zero_index = src[0];

    for (int i = 1; i < len; i++) {
        if (i == next_zero_index) {
            next_zero_index += src[i];
            dst[i - 1] = 0;
        } else {
            dst[i - 1] = src[i];
        }
    }

    return;
}


void CobsEncodedCommunicationBase::add_header(uint8_t *dst, uint8_t *src, uint8_t len, uint8_t id) {
    dst[0] = id; // ID
    dst[1] = len; // データ長
    dst[2] = calc_checksum(src, len); // チェックサム
    memcpy(dst + HEADER_LENGTH, src, len); // データ
    return;
}

uint8_t CobsEncodedCommunicationBase::calc_checksum(uint8_t *src, uint8_t len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += src[i];
    }
    return checksum;
}

void CobsEncodedCommunicationBase::remove_header(uint8_t *dst, uint8_t *src, uint8_t len, uint8_t *id, bool *error) {
    *id = src[0];
    uint8_t data_len = src[1];
    uint8_t checksum = src[2];
    memcpy(dst, src + HEADER_LENGTH, data_len);
    *error = (checksum != calc_checksum(dst, data_len));
    return;
}

void CobsEncodedCommunicationBase::send(uint8_t frame_id, uint8_t* data, size_t size) {
    uint8_t send_buffer[MAX_FRAME_LENGTH];
    add_header(send_buffer, data, size, frame_id); // ヘッダの追加
    uint8_t encoded_buffer[MAX_FRAME_LENGTH];
    encode_cobs(encoded_buffer, send_buffer, size + HEADER_LENGTH); // COBS のエンコード
    send_raw(encoded_buffer, size + HEADER_LENGTH + 2);
}

void CobsEncodedCommunicationBase::on_receive_raw(uint8_t* data, uint8_t size) {
    uint8_t decoded_buffer[MAX_FRAME_LENGTH];
    uint8_t header_removed_buffer[MAX_FRAME_LENGTH];
    decode_cobs(decoded_buffer, data, size);
    uint8_t id;
    bool error = false;
    remove_header(header_removed_buffer, decoded_buffer, size - 2, &id, &error);
    if (!error) {
        on_receive(id, header_removed_buffer, size - 2);
    }
}