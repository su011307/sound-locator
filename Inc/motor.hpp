#pragma once

#include <cstdint>
#include <array>
#include <stm32f4xx.h>
#include "driver.hpp"

constexpr uint8_t GEAR_RATIO = 34;         // 减速比
constexpr uint16_t PULSES_PER_ROUND = 500; // 电机每转一圈产生的脉冲数
constexpr float STOP_THRESHOLD_ANGLE = 1.0f;
constexpr uint32_t DELTA = 100 * 1000; // 时间间隔，单位：us
constexpr uint32_t MAX_PWM = 1999;     // PWM的最大值

/*
 * @brief PID控制的相关参数
 */
struct PIDParams
{
    float Kp, Ki, Kd;
    float integral;
    float last_error;
};

class Encoder
{
    // @bug 可能没有正确处理计时器初始化的问题
public:
    explicit Encoder(TIM_HandleTypeDef *htim);
    // @brief 获取当前总脉冲数
    [[nodiscard]] uint32_t get_pulse_count() const;
    // @brief 重置编码器
    void reset() const;

private:
    TIM_HandleTypeDef *htim_;
};

// [DISCLAMER] 在构建电机抽象的时候使用了Qwen查询并解释电机的性能参数
// 并查阅了有关的HAL API的使用细节
/*
 * @brief 一个抽象的电机
 * @param
 */
class Motor {
public:
    Motor(
        TIM_HandleTypeDef *pwm_htim,
        uint32_t pwm_channel,
        GPIO *ain1,
        GPIO *ain2,
        Encoder *encoder,
        PIDParams *param);

    void poll();
    void move_to(float abs_angle);
    void stop();
    [[nodiscard]] float get_angle() const;
    [[nodiscard]] bool is_busy() const;

private:
    void set_pwm(uint32_t pwm_value) const;
    void set_direction(bool forward) const;

    bool is_moving_;
    float target_angle_;
    float curr_ang_{};
    uint16_t last_pulse_cnt_{};

    const uint8_t gear_ratio_;
    const uint16_t pulses_per_round_;
    const uint16_t delta_;

    TIM_HandleTypeDef *pwm_htim_;
    Encoder *encoder_;
    uint32_t pwm_channel_;
    GPIO *ain1_; // TB6612的AIN1引脚，控制电机正向转动
    GPIO *ain2_; // TB6612的AIN2引脚，控制电机反向转动
    PIDParams *param_;
};