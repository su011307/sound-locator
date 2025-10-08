#pragma once
#include <cstdint>
#include <memory>
#include <array>
#include "filter/filter.hpp"

// 麦克风工作模式
enum class Mode: uint8_t{
    Single,
    Continuous,
    Measure,
    Random
};

class Microphone{
public:
    void update(uint16_t raw_adc, uint32_t timestamp);
    void reset();
    void change_mode();
private:
    std::unique_ptr<KalmanFilter> filter_;
    uint32_t adc_channel_;
};

class MicrophoneMatrix{
public:
    void start();
    void callback();
private:
    ADC_HandleTypeDef* hadc_;
    TIM_HandleTypeDef* timer_;
    std::unique_ptr<Microphone> mic0_;
    std::unique_ptr<Microphone> mic1_;
    std::unique_ptr<Microphone> mic2_;
    std::unique_ptr<Microphone> mic3_;
    std::array<uint32_t, 4> buffer_;
    Mode mode_;
    bool is_ready_;
    void check_is_ready();
};

