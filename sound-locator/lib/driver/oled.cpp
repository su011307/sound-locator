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

// SSD1306 初始化命令序列
static const std::array<uint8_t, 28> SSD1306_INIT_CMDS = {
    0xAE, // Display off
    0x20, 0x00, // Set Memory Addressing Mode: Horizontal
    0xB0, // Set Page Start Address for Page Addressing Mode
    0xC8, // COM Output Scan Direction: remapped mode
    0x00, // Set low column address
    0x10, // Set high column address
    0x40, // Set start line address
    0x81, 0x7F, // Set contrast control
    0xA1, // Set segment re-map 0 to 127
    0xA6, // Set normal display
    0xA8, 0x3F, // Set multiplex ratio(1 to 64)
    0xA4, // Output follows RAM content
    0xD3, 0x00, // Set display offset
    0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
    0xD9, 0xF1, // Set pre-charge period
    0xDA, 0x12, // Set com pins hardware configuration
    0xDB, 0x40, // Set vcomh
    0x8D, 0x14, // Enable charge pump regulator
    0xAF // Display ON
};

OLED::OLED(I2CPort &i2c_port) : i2c_(i2c_port) {
    buffer_.fill(0);
}

bool OLED::init() {
    is_initialized_ = send_commands(SSD1306_INIT_CMDS);
    clear();
    refresh();
    return is_initialized_;
}

void OLED::clear() {
    buffer_.fill(0);
}

void OLED::draw_string(uint8_t x, uint8_t y, const char *str) {
    uint8_t orig_x = x;
    while (*str) {
        if (x + 8 > WIDTH) {
            x = orig_x;
            y += 16;
        }
        if (y + 16 > HEIGHT) break;
        draw_char(x, y, *str);
        x += 8;
        ++str;
    }
}

bool OLED::refresh() {
    for (uint8_t page = 0; page < PAGES; ++page) {
        std::vector<uint8_t> cmds = {
            static_cast<uint8_t>(0xB0 | page), // Set page address
            static_cast<uint8_t>(0x00),        // Set lower column address
            static_cast<uint8_t>(0x10)         // Set higher column address
        };
        if (!send_commands(cmds)) return false;
        // 每页128字节
        if (!send_data(&buffer_[page * WIDTH], WIDTH)) return false;
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

