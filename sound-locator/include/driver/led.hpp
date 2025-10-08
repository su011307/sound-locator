#pragma once
#include <cstdint>
#include <array>
#include <abstraction/driver.hpp>

class LEDMatrix
{
public:
    LEDMatrix(GPIO *a0, GPIO *a1, GPIO *a2, GPIO *e1);

    void init();
    void light_up(uint16_t degree);
    
private:
    GPIO *a0_;
    GPIO *a1_;
    GPIO *a2_;
    GPIO *e1_;
};