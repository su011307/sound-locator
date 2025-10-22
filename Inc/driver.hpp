#pragma once

#include <cstdint>
#include <stm32f4xx.h>

class GPIO{
public:
    GPIO(GPIO_TypeDef *port, uint16_t pin);
    void set_high();
    void set_low();
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