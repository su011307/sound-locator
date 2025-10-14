#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <array>

class I2CPort; // 前置声明

class OLED{
public:
    static constexpr uint8_t WIDTH = 128;
    static constexpr uint8_t HEIGHT = 64;
    static constexpr uint8_t PAGES = HEIGHT / 8;
    static constexpr uint8_t SSD1306_I2C_ADDR = 0x78;
    static constexpr uint8_t CHAR_WIDTH = 8;
    static constexpr uint8_t CHAR_HEIGHT = 16;

    OLED(I2CPort &i2c_port);
    bool init();

    /*
     * @brief 向屏幕中写入内容
     * @return `bool` 是否写入成功
     */
    bool write(const std::string& content);
    void clear();
    bool refresh();
private:
    void draw_pixel(uint8_t x, uint8_t y, bool on = true);
    void draw_char(uint8_t x, uint8_t y, char c);
    bool send_command(uint8_t cmd);
    bool send_commands(const std::vector<uint8_t> &cmds);
    bool send_data(const uint8_t *data, size_t len);
    std::array<uint8_t, WIDTH * PAGES> buffer_;
    I2CPort &i2c_;
    bool is_initialized_ = false;
};

