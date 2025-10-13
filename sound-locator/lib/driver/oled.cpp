/*
 * @brief 这个文件提供了OLED的抽象，用来操作显示屏
 * [DISCLAIMER] 在AI代码的基础上自己写的，不然根本不知道Qwen在写什么，心里不踏实
 */
#include "driver/oled.hpp"
#include "abstraction/i2c.hpp"
#include <cstring>
#include <algorithm>
#include <array>
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>
#include "font.cpp" // 直接用 font_map

// SSD1306 初始化命令序列。这些指令我没有查文档复核
static const std::vector<uint8_t> SSD1306_INIT_CMDS = {
    0xAE,       // Display OFF
    0xD5, 0x80, // Set Display Clock Divide Ratio/Oscillator Frequency
    0xA8, 0x3F, // Set Multiplex Ratio
    0xD3, 0x00, // Set Display Offset
    0x40,       // Set Display Start Line
    0x8D, 0x14, // Charge Pump Setting
    0x20, 0x00, // Set Memory Addressing Mode - Horizontal
    0xA1,       // Set Segment Re-map
    0xC8,       // Set COM Output Scan Direction
    0xDA, 0x12, // Set COM Pins Hardware Configuration
    0x81, 0xCF, // Set Contrast Control
    0xD9, 0xF1, // Set Pre-charge Period
    0xDB, 0x40, // Set VCOMH Deselect Level
    0xA4,       // Entire Display ON from RAM
    0xA6,       // Set Normal Display
    0xAF        // Display ON
};

OLED::OLED(I2CPort &i2c_port) : i2c_(i2c_port) {
    buffer_.fill(0);
}

bool OLED::init() {
    is_initialized_ = send_commands(SSD1306_INIT_CMDS);
    if (is_initialized_){
        clear();
        refresh();
    }
    return is_initialized_;
}

void OLED::clear() {
    buffer_.fill(0);
}

void OLED::draw_pixel(uint8_t x, uint8_t y, bool on){
    if (x >= WIDTH || y >= HEIGHT){
        return;
    }
    uint8_t page = y / 8;
    uint8_t bit_pos = y % 8;
    uint32_t index = page * WIDTH + x;

    if (on) {
        buffer_[index] |= (1 << bit_pos);
    } else {
        buffer_[index] &= ~(1 << bit_pos);
    }
}

void OLED::draw_char(uint8_t x, uint8_t y, char c) {
    auto it = font_map.find(c);
    if (it == font_map.end()){
        return;
    }

    const Font& font_data = it->second;

    uint8_t page1 = y / 8;
    uint8_t page2 = (y + 8) / 8;

    if (page2 >= PAGES || x + CHAR_WIDTH > WIDTH){
        return;
    }

    for (int i = 0; i < 8; i++){
        buffer_[(page1 * WIDTH) + x + i] = font_data[i];
        buffer_[(page2 * WIDTH) + x + i] = font_data[i];
    }
}

bool OLED::write(const std::string& content){
    // 1. 验证内容是否有效
    // 计算屏幕容量
    const size_t max_chars_per_line = WIDTH / CHAR_WIDTH; // 128 / 8 = 16
    const size_t max_lines = HEIGHT / CHAR_HEIGHT;        // 64 / 16 = 4
    const size_t screen_capacity = max_chars_per_line * max_lines;

    // 检查内容是否过长
    if (content.length() > screen_capacity)
    {
        return false;
    }

    // 检查是否所有字符都在字体库中
    for (char c : content)
    {
        if (font_map.find(c) == font_map.end())
        {
            return false; // 包含未知字符
        }
    }

    // 2. 如果验证通过，则开始绘制
    clear(); // 清空屏幕缓冲区

    uint8_t current_x = 0;
    uint8_t current_y = 0;

    for (char c : content)
    {
        // 如果当前行写不下了，就换行
        if (current_x + CHAR_WIDTH > WIDTH)
        {
            current_x = 0;
            current_y += CHAR_HEIGHT;
        }

        // 绘制字符
        draw_char(current_x, current_y, c);

        // 移动光标
        current_x += CHAR_WIDTH;
    }

    // 3. 刷新屏幕以显示内容
    return refresh();
}

bool OLED::refresh()
{
    for (uint8_t page = 0; page < PAGES; ++page)
    {
        std::vector<uint8_t> cmds = {
            static_cast<uint8_t>(0xB0 | page), // Set page address
            static_cast<uint8_t>(0x00),        // Set lower column address
            static_cast<uint8_t>(0x10)         // Set higher column address
        };
        if (!send_commands(cmds))
            return false;

        std::vector<uint8_t> page_data;
        page_data.assign(&buffer_[page * WIDTH], &buffer_[page * WIDTH] + WIDTH);
        // 每页128字节
        if (!i2c_.write_data(SSD1306_I2C_ADDR, page_data))
            return false;
    }
    return true;
}

bool OLED::send_command(uint8_t cmd) {
    return i2c_.write_command(SSD1306_I2C_ADDR, {cmd});
}

bool OLED::send_commands(const std::vector<uint8_t> &cmds) {
    return i2c_.write_command(SSD1306_I2C_ADDR, cmds);
}

bool OLED::send_data(const uint8_t *data, size_t len) {
    std::vector<uint8_t> buf(data, data + len);
    return i2c_.write_data(SSD1306_I2C_ADDR, buf);
}
