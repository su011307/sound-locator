//
// Created by 24842 on 25-10-21.
//
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <stm32f4xx_hal.h>

using std::string, std::optional, std::nullopt, std::array;
constexpr uint16_t BUFFER_SIZE = 64;

extern UART_HandleTypeDef huart6;

void MX_USART6_UART_Init(void);

class Transmitter {
public:
    explicit Transmitter(UART_HandleTypeDef *huart);
    void send(string message, const optional<uint32_t> &timeout) const;
private:
    UART_HandleTypeDef *huart_;
    array<uint8_t, BUFFER_SIZE> buffer_{};
};
