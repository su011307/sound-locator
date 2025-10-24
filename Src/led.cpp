#include "led.hpp"

LEDMatrix::LEDMatrix(GPIO *a0, GPIO *a1, GPIO *a2)
    : a0_(a0), a1_(a1), a2_(a2){ }

void LEDMatrix::init() const {
    a0_->set_low();
    a1_->set_low();
    a2_->set_low();
}

void LEDMatrix::light_up(const uint16_t init_degree){
    // init();
    if (const auto degree = init_degree % 3600; degree < 225 || degree >= 3375)
    {
        // 位置处在第0个扇区覆盖范围内
    }
    else
    {
        // 位置不在第0个扇区覆盖范围内
        uint8_t group = 1;
        while (degree > 225 + 450u * group)
        {
            group++;
        }

        // 懒得写函数映射，直接穷举了
        switch (group)
        {
        case 1:
            a0_->set_high();
            break;

        case 2:
            a1_->set_high();
            break;

        case 3:
            a0_->set_high();
            a1_->set_high();
            break;

        case 4:
            a2_->set_high();
            break;

        case 5:
            a0_->set_high();
            a2_->set_high();
            break;

        case 6:
            a1_->set_high();
            a2_->set_high();
            break;

        case 7:
            a0_->set_high();
            a1_->set_high();
            a2_->set_high();
            break;

        default:
            break;
        }
    }
}