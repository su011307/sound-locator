#pragma once

#include <cstdint>
#include <array>
#include "driver.hpp"

class LEDMatrix
{
public:
    LEDMatrix(GPIO *a0, GPIO *a1, GPIO *a2);

    // @brief 初始化灯环
    void init();
    // @brief `degree`必须是`[0, 3600)`以内的数(单位: `0.1°`)
    // @warning 这个成员函数没有运行时检查，请务必确保输入合法
    void light_up(uint16_t degree);
    
private:
    GPIO *a0_;
    GPIO *a1_;
    GPIO *a2_;
};