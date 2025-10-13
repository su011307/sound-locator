#include "driver/led.hpp"

LEDMatrix::LEDMatrix(GPIO *a0, GPIO *a1, GPIO *a2, GPIO *e1)
    : a0_(a0), a1_(a1), a2_(a2), e1_(e1){ }

void LEDMatrix::init(){
    a0_->set_low();
    a1_->set_low();
    a2_->set_low();
    e1_->set_low();
}

void LEDMatrix::light_up(uint16_t degree){
    init();
    if (degree < 225 || degree >= 3375)
    {
        // 位置处在第0个扇区覆盖范围内
        e1_->set_high();
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
            e1_->set_high();
            a0_->set_high();
            break;

        case 2:
            e1_->set_high();
            a1_->set_high();
            break;

        case 3:
            e1_->set_high();
            a0_->set_high();
            a1_->set_high();
            break;

        case 4:
            e1_->set_high();
            a2_->set_high();
            break;

        case 5:
            e1_->set_high();
            a0_->set_high();
            a2_->set_high();
            break;

        case 6:
            e1_->set_high();
            a1_->set_high();
            a2_->set_high();
            break;

        case 7:
            e1_->set_high();
            a0_->set_high();
            a1_->set_high();
            a2_->set_high();
            break;

        default:
            break;
        }
    }
}