#pragma once

#include <mbed.h>

class BNO055
{
    public:
        UnbufferedSerial serial;

        BNO055(PinName tx=USBTX, PinName rx=USBRX, std::chrono::milliseconds internal=20ms);

        //度数法で値を取得
        float getDegrees();

        //弧度法で値を取得
        float getRadians();

        //現在の方向をゼロとする．
        void reset(int value = 0);

        //弧度法で値を設定
        void setRadians(float radians);

        bool active();

        
    private:
        Ticker ticker;

        Timer timer;

        //値をリクエスト
        void requestValue();
        //受信割り込み
        void interrupt();

        //レジスタ1バイト分を書き込む
        void writeByte(unsigned char reg_add, unsigned char value);

        //値を更新
        void updateValue();
        
        //現在の値（生の値）
        int16_t raw_value = 0;

        //零点
        int zero_point = 0;

        //
        int getValue();
        

        //分解能
        const int RESOLUTION = 5760;

        //送信
        const char START_BYTE = 0xAA; 
        const char RESPONSE_BYTE = 0xBB;
        const char WRITE_BYTE = 0x00;
        const char READ_BYTE = 0x01;

        //EULER角が保持されたレジスタの先頭のアドレス
        const char TARGET_REGISTER = 0x1A;

        //受信バッファー
        char receive_buffer[16];

        //受信プロセス
        enum RECEIVE_PROCESS{
            STARTING, //0xBBを受信
            SET_LENGTH, //長さを受信
            RECEIVING //値を受信
        };

        int receive_process = STARTING;

        unsigned int receive_index = 0;
        unsigned int receive_length = 0;

        bool _active = false;


        const std::chrono::milliseconds CONNECTION_TIMEOUT = 1000ms;
};