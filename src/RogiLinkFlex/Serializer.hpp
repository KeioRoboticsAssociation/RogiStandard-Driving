#pragma once

#include <type_traits>
#include <mbed.h>
#include <vector>

// ------ シリアライズ ------

// 基本型のシリアライズ
template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value>::type
serialize(uint8_t* buffer, size_t* size, const T data){
    memcpy(buffer, &data, sizeof(T));
    *size = sizeof(T);
};

// char*のシリアライズ
template <typename T>
typename std::enable_if<std::is_same<T, char*>::value>::type
serialize(uint8_t* buffer, size_t* size, const T data){
    memcpy(buffer, data, strlen(data) + 1);
    *size = strlen(data) + 1;
};

// std::vectorのシリアライズ
template <typename T>
void serialize(uint8_t* buffer, size_t* size, const std::vector<T>& data){
    serialize(buffer, 1, data.size()); // 長さを格納
    int offset = 1;
    for (const T& d : data) {
        uint8_t s;
        serialize(buffer + offset, &s, d); // データを格納
        offset += s;
    }
    *size = offset;
};

// user-defined typeのシリアライズ
template <typename T>
typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_same<T, char*>::value && !std::is_same<T, std::vector<typename T::value_type> >::value>::type
serialize(uint8_t* buffer, size_t* size, const T data){
    data.serialize(buffer, size); // serialize関数が定義されていることを期待
};

// 可変長引数の場合
template <typename T, typename... Args>
void serialize(uint8_t* buffer, size_t* size, const T data, Args... args){
    size_t s1, s2;
    serialize(buffer, &s1, data);
    serialize(buffer + s1, &s2, args...);
    *size = s1 + s2; 
};

// ------ デシリアライズ ------

// 基本型のデシリアライズ
template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value>::type
deserialize(const uint8_t* buffer, T* data, size_t* size){
    memcpy(data, buffer, sizeof(T));
    *size = sizeof(T);
};

// char*のデシリアライズ
template <typename T>
typename std::enable_if<std::is_same<T, char*>::value>::type
deserialize(const uint8_t* buffer, T* data, size_t* size){
    *data = (char*)buffer;
    *size = strlen((char*)buffer) + 1;
};

// std::vectorのデシリアライズ
template <typename T>
void deserialize(const uint8_t* buffer, std::vector<T>* data, size_t* size){
    size_t len;
    deserialize(buffer, &len, size); // 長さを取得
    int offset = 1;
    for (int i = 0; i < len; i++) {
        T d;
        size_t s;
        deserialize(buffer + offset, &d, &s); // データを取得
        data->push_back(d);
        offset += s;
    }
    *size = offset;
};

// user-defined typeのデシリアライズ
template <typename T>
typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_same<T, char*>::value && !std::is_same<T, std::vector<typename T::value_type> >::value>::type
deserialize(const uint8_t* buffer, T* data, size_t* size){
    data->deserialize(buffer, size); // deserialize関数が定義されていることを期待
};

// 可変長引数の場合
template <typename T, typename... Args>
void deserialize(const uint8_t* buffer, T* data, Args*... args){
    size_t size;
    deserialize(buffer, data, &size);
    deserialize(buffer + size, args...);
};

template<typename... Args>
class TupleDeserializer{
    public:
    int deserialize_tuple(const uint8_t* buffer, std::tuple<Args...>& tuple, size_t len) {
        return deserialize_tuple_impl(buffer, std::index_sequence_for<Args...>(), len, tuple);
    }

    template<std::size_t... I>
    int deserialize_tuple_impl(const uint8_t* buffer, std::index_sequence<I...>, size_t len, std::tuple<Args...>& tuple) {
        size_t offset = 0;

        // タプルの各要素を順番にデシリアライズする。std::tupleの各要素に対して実行するのにfor文が使えないため、可変長テンプレートとinitializer_listを無理やり使っている。
        (void)std::initializer_list<int>{([&](){
            if (offset > len) {
                return 0;
            }

            size_t size;
            deserialize(buffer + offset, &std::get<I>(tuple), &size);
            offset += size;
            return 0;
        }())...};

        if (offset != len) {
            return 1;
        }

        return 0;
    }  
};


