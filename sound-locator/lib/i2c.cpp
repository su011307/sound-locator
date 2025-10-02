/*
 * I2C协议的抽象。因为整个项目中唯一要用到I2C的就是OLED屏幕
 * 完全没有接收需求，所以这个抽象特别简陋，主打一手能用就行
 */
#include <cstdint>
#include <vector>
#include <stm32f4xx.h>

/*
 * @brief 这是一个I2C通讯数据包的抽象
 * @details I2C的通讯数据包由三部分组成，头部是7位的地址
 * 后面跟着一个字节的控制字节（Control Byte）和一个字节的
 * 数据字节（Data Byte）
 * 不过话说回来，好像不一定非得一次一个字节，因为HAL的函数签名中
 * 有一个Size参数，应该是可以一次性发送多个字节的
 * 具体细节请参阅：https://blog.csdn.net/Fine_rose/article/details/114026892
 * @param address `uint8_t` 从机的地址，SSD1306一般是0x78。7位，注意位移1位
 * @param control `uint8_t` 控制字节，SSD1306一般是0x40或者0x00
 * @param data `vector<uint_8>` 需要传输的数据
 */
struct I2CPackage{
    uint8_t address;
    uint8_t control;
    std::vector<uint8_t> data;
};

class I2CPort{
    public:
        I2CPort(I2C_HandleTypeDef* i2c);
        void SendPackage(const I2CPackage& package);
    private:
        I2C_HandleTypeDef* i2c_;
        uint8_t address_;
        uint32_t timeout_ = 500;
};

I2CPort::I2CPort(I2C_HandleTypeDef* i2c)
    : i2c_(i2c) {

    }

/*
 * @brief 向I2C的从机发送数据包
 */
void I2CPort::SendPackage(const I2CPackage& package){
    std::vector<uint8_t> buff;
    HAL_I2C_Master_Transmit(
        i2c_, static_cast<uint16_t>(address_),
        const_cast<uint8_t*> (&package.control),
        package.data.size(), timeout_
    );
}