#pragma once

#include <cstdint>
#include <memory>
#include <array>
#include <stm32f4xx.h>
#include "filter.hpp"

constexpr uint8_t MIC_NUMBER = 4;
constexpr uint16_t FREQUENCY = 1000;
constexpr uint16_t MEAN = 2048;
constexpr float TRIGGER_TRHESH = 3096.0f;

// @brief 当前测量的工作模式
enum class Mode : uint8_t
{
    Single,     // 单次定位，正弦波
    Continuous, // 连续追踪，正弦波
    Measure,    // 测量距离，正弦波
    Random      // 播放最炫民族风，确定声源位置，单次
};

class Microphone{
public:
    Microphone(uint32_t adc_channel);
    // @brief 用来更新数值，这个数值会经过滤波器过滤
    void update(uint16_t raw_adc, uint32_t timestamp);
    // @brief 重置一个麦克风
    void reset();
    // @brief 当前的麦克风是否被触发了。这个成员函数只适用于TDoA模式
    bool is_triggered() const;
private:
    KalmanFilter filter_;
    uint32_t adc_channel_;

    bool triggered_;
    std::optional<uint32_t> trig_time_;
    std::optional<uint32_t> reach_zero_;
    float last_value_;
};

/*
 * @warning 这个麦克风阵列并没有校验是否进行了初始化（start）。
 * 请务必先调用start函数，不然这个阵列无法发挥正常的作用
 */
class MicrophoneMatrix{
public:
    MicrophoneMatrix(ADC_HandleTypeDef *hadc,
                     TIM_HandleTypeDef *timer);
    // @brief 麦克风阵列，启动！
    void start();

    // @bug 如果ADC采样次数过高，CPU来不及处理，缓冲区会不会溢出？
    // @brief 回调函数，供中断触发后调用
    void callback();
    // @brief 改变麦克风的工作模式
    void change_mode();

    bool is_ready() const;

    Mode get_mode() const;

    const std::array<uint32_t, MIC_NUMBER> &get_timestamps() const;

    MicrophoneMatrix(const MicrophoneMatrix&) = delete;
    MicrophoneMatrix& operator=(const MicrophoneMatrix&) = delete;
private:
    ADC_HandleTypeDef* hadc_;
    TIM_HandleTypeDef* timer_;
    std::array<Microphone, MIC_NUMBER> microphones_;
    std::array<uint32_t, MIC_NUMBER> buffer_;
    Mode mode_;
    bool is_ready_;
    // @details 这个成员函数只在TDoA的模式下可用
    void check_if_ready();
};

