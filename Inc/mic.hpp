#pragma once

#include <stm32f4xx.h>
#include <array>
#include <optional>
#include <utility>
#include <memory>
#include "filter.hpp"

constexpr uint32_t ADC_SAMPLING_SPAN = 1000; // 麦克风矩阵ADC采样的间隔，单位：us
constexpr uint8_t FILTER_PERIOD = 10;        // ZLEMA的周期
constexpr uint32_t TRIGGER_THRESH = 1024;    // 触发阈值
constexpr uint32_t BUFFER = 24; // 留一个中间地带，防止去噪不彻底的抖动
constexpr uint32_t WINDOW = 600; // 单位: us
constexpr uint32_t GUARANTEE = 800; // 单位: us

constexpr uint32_t U32M = ~static_cast<uint32_t>(0); // ~0u即2^32-1

enum class WorkingMode : uint8_t
{
    Single,
    Random,
    Measure,
    Continuous
};

class MicrophoneMatrix
{
public:
    MicrophoneMatrix();
    uint32_t *dma_ptr() {return mic_buffer_.data();}
    [[nodiscard]] bool is_ok() const;
    void switch_mode();
    void update(uint32_t timestamp);
    [[nodiscard]] std::array<uint32_t, 4> get_timestamps() const;

private:
    WorkingMode mode_;

    std::optional<uint32_t> gate_open_time_;
    std::optional<bool> gate_direction_; // true为上穿窗口，false为下穿窗口
    std::optional<bool> last_direction_; // 最近一次有效触发方向

    std::array<uint32_t, 4> mic_buffer_;
    std::array<std::unique_ptr<ZLEMAFilter>, 4> filters_;

    std::array<
        std::pair<
            std::optional<uint32_t>, // 上穿
            std::optional<uint32_t>>, // 下穿
        4>
        timestamp_records_;

    std::array<uint32_t, 4> last_value_;
};