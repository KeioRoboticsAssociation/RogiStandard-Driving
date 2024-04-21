#pragma once
#include "DCMotor/DCMotor.hpp"
#include "Encoder/Encoder.hpp"

template <typename T>
class RingQueue {
    public:
    RingQueue(int size) : size(size) {
        this->array = new T[size];
        this->count = 0;
    }

    ~RingQueue() {
        delete[] this->array;
    }

    T pop(T value) {
        T result = this->array[this->count];
        this->array[this->count] = value;
        this->count = (this->count + 1) % this->size;
        return result;
    }

    public:
        T* array;
        int size;
        int count;
};

class MotorSimulator {
public:
    MotorSimulator(DCMotor& motor, Encoder* encoder, float gain=100.0, float timeConstant=0.0, float deadTime=0.0, float interval=10);
    void loop();
    float getSpeed();

    ~MotorSimulator();

private:
    DCMotor& motor;
    Encoder* encoder;

    float interval;

    float constantA;
    float constantB;

    float last_output;
    float output;

    RingQueue<float> queue;
};