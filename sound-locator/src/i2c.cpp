/*
 * [DISCLAIMER]
 * 这些代码在在检查阶段使用了Qwen来检查错误，并按照其指示修改和完善了代码
 * 
 * I2C协议的抽象。因为整个项目中唯一要用到I2C的就是OLED屏幕
 * 完全没有接收需求，所以这个抽象特别简陋，主打一手能用就行
 */
#include <cstdint>
#include <vector>
#include <stm32f4xx.h>
#include "abstraction/i2c.hpp"


class I2CPort{
    public:
        I2CPort(I2C_HandleTypeDef* hi2c);

        // @brief 向从机发送指令
        bool write_command(uint8_t address, const std::vector<uint8_t>& commands){
            std::vector<uint8_t> buffer;
            buffer.reserve(1 + commands.size());
            buffer.push_back(0x00);
            buffer.insert(buffer.end(), commands.begin(), commands.end());
            bool is_ok = HAL_I2C_Master_Transmit(
                hi2c_, 
                address, 
                buffer.data(), 
                buffer.size(), 
                timeout
            ) == HAL_OK;

            return is_ok;
        }

        // @brief 向从机发送数据包
        bool write_data(uint8_t address, const std::vector<uint8_t>& datas){
            if (datas.empty()) return true;
            std::vector<uint8_t> buffer;
            buffer.reserve(1 + datas.size());
            buffer.push_back(0x40);
            buffer.insert(buffer.end(), datas.begin(), datas.end());
            bool is_ok = HAL_I2C_Master_Transmit(
                hi2c_,
                address,
                buffer.data(),
                buffer.size(),
                timeout
            ) == HAL_OK;

            return is_ok;
        };
    private:
        I2C_HandleTypeDef* hi2c_;
        uint8_t address_;
        static constexpr uint32_t timeout = 500;
};
