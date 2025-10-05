#include "abstraction/driver.hpp"


class Timer{
    public:
        Timer(TIM_HandleTypeDef* htim);
        int32_t count() const;
        void reset();
    private:
        TIM_HandleTypeDef* htim_;
};

Timer::Timer(TIM_HandleTypeDef* htim)
    : htim_(htim){}

// [DISCLAIMER] 为了免去查手册的麻烦，这两个成员函数的HAL API都是问Qwen AI的
int32_t Timer::count() const {
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_));
}

void Timer::reset(){
    __HAL_TIM_SET_COUNTER(htim_, 0);
}


class Microphone{
    public:
        Microphone(ADC_HandleTypeDef* hadc);
        CircleBuffer buff;

        bool triggered(uint8_t mic_id) const;
        uint16_t adx_value(uint8_t mic_id) const;
    private:
        ADC_HandleTypeDef* hadc_;

        uint16_t adc_buffer_[5];

}