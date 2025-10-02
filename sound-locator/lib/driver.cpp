/*
 * @brief 这个文件把要用到的四个引脚抽象成类了
 * 为了保护我脆弱的大脑和所剩无几的san值
 */
#pragma once
#include <stm32f4xx_hal.h>

using Pin = uint16_t;


/*
 * [DISCLAIMER]
 * 这里问了Qwen AI怎么在C++中使用类似于Rust中的枚举
 * 看得出来我真的很爱Rust，也真的不是很熟悉C++🥹
 */

enum class GPIOInputMode: uint8_t{
    Floating,
    PullUp,
    PullDown
}

enum class GPIOOutputMode: uint8_t{

}

class GPIO{
    public:
        GPIO(GPIO_TypeDef* config, Pin pin);
        const GPIOInputMode input_mode;
        const GPIOOutputMode output_mode;

        bool status;

        void init();
        void reverse();
        bool read() const;

    private:
        GPIO_TypeDef* config_;
        Pin pin_;
};

class I2C{
    public:
    private:
};

class PWM{
    public:
    private:
};

class ADC{
    public:
    private:
};