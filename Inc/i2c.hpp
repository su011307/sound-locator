#pragma once
#include <cstdint>
#include <vector>
#include <stm32f4xx.h>
#include "main.hpp"

void MX_I2C1_Init(void);

extern I2C_HandleTypeDef hi2c1;

class I2CPort{
public:
    I2CPort(I2C_HandleTypeDef* hi2c);
    // @brief 向从机发送指令
    bool write_command(uint8_t address, const std::vector<uint8_t> &commands);
    // @brief 向从机发送数据包
    bool write_data(uint8_t address, const std::vector<uint8_t> &datas);

private:
    I2C_HandleTypeDef* hi2c_;
    static constexpr uint32_t timeout = 10;
};

