#pragma once
#include <stm32f4xx_hal.h>

class GPIO{
public:
    GPIO(GPIO_TypeDef *port, uint16_t pin);
    void set_high();
    void set_low();
    void reverse();
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

class Microphone{
public:
    Microphone(ADC_HandleTypeDef* hadc);
    //CircleBuffer buff;
    bool triggered(uint8_t mic_id) const;
    uint16_t adx_value(uint8_t mic_id) const;
private:
    ADC_HandleTypeDef* hadc_;
    uint16_t adc_buffer_[5];
};

