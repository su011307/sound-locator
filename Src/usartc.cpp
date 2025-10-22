//
// Created by 24842 on 25-10-21.
//
#include "main.hpp"
#include "usart.hpp"

UART_HandleTypeDef huart6;

static void MX_USART6_UART_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void MX_USART6_UART_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    MX_USART6_UART_GPIO_Init();

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}

Transmitter::Transmitter(UART_HandleTypeDef *huart) : huart_(huart)
{

}

void Transmitter::send(string message, const optional<uint32_t> &timeout) const {
    const auto time = timeout.has_value() ? timeout.value() : 50;
    auto *msg = message.data();
    if (const auto len = message.size(); HAL_UART_Transmit(huart_, reinterpret_cast<uint8_t*>(msg), len, time) != HAL_OK) {
        Error_Handler();
    }
}

