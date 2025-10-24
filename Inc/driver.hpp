#pragma once

#include <cstdint>
#include <stm32f4xx.h>

class GPIO{
public:
    GPIO(GPIO_TypeDef *port, uint16_t pin);
    void set_high() const;
    void set_low() const;
private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

class Timer{
public:
    Timer(TIM_HandleTypeDef* htim);
    int32_t count() const;
    void reset();
private:
    TIM_HandleTypeDef* htim_;
};