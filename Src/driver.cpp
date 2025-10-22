#include "driver.hpp"

GPIO::GPIO(GPIO_TypeDef *port, uint16_t pin)
    : port_(port), pin_(pin) {}

void GPIO::set_high()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

void GPIO::set_low()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

Timer::Timer(TIM_HandleTypeDef *htim)
    : htim_(htim) {}

// [DISCLAIMER] 为了免去查手册的麻烦，这两个成员函数的HAL API都是问Qwen AI的
int32_t Timer::count() const
{
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_));
}

void Timer::reset()
{
    __HAL_TIM_SET_COUNTER(htim_, 0);
}
