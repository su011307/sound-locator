#include <cstdint>
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
