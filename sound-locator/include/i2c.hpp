#pragma once
#include <cstdint>
#include <vector>
#include <stm32f4xx.h>

class I2CPort{
public:
    I2CPort(I2C_HandleTypeDef* hi2c);
    bool write_command(uint8_t address, const std::vector<uint8_t>& commands);
    bool write_data(uint8_t address, const std::vector<uint8_t>& datas);
private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t address_;
    static constexpr uint32_t timeout = 500;
};

