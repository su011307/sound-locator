#pragma once

#include <cstdint>
#include <array>
#include <stm32f4xx.h>
#include "abstraction/driver.hpp"

constexpr uint8_t GEAR_RATIO = 34;         // 减速比
constexpr uint16_t PULSES_PER_ROUND = 500; // 电机每转一圈产生的脉冲数
constexpr uint16_t DELTA = 100;            // 时间间隔，单位：us
constexpr uint32_t MAX_PWM = 1999;           // PWM的最大值

/*
 * @brief PID控制的相关参数
 */
struct PIDParams
{
    const float Kp;
    const float Ki;
    const float Kd;

    float intergral;
    float diveritive;
    float last_error;
};

class Encoder
{
    // @bug 可能没有正确处理计时器初始化的问题
public:
    Encoder(TIM_HandleTypeDef *htim, uint16_t delta_t);
    // @brief 获取当前总脉冲数
    uint32_t get_pulse_count() const;
    // @brief 重置编码器
    void reset();

private:
    uint8_t gear_ratio_;
    uint16_t delta_t_;
    TIM_HandleTypeDef *htim_;
};

// [DISCLAMER] 在构建电机抽象的时候使用了Qwen查询并解释电机的性能参数
// 并查阅了有关的HAL API的使用细节
/*
 * @brief 一个抽象的电机
 * @param
 */
class Motor
{
public:
    Motor(
        TIM_HandleTypeDef *pwm_htim,
        uint32_t pwm_channel,
        GPIO *ain1,
        GPIO *ain2,
        Encoder *encoder,
        PIDParams *param);

    // @brief 获取当前电机指向的角度
    float current_angle() const;
    // @brief 按照单位圆内绝对角度更新位置
    void abs_update(float target_ang);
    // @brief 按照角度的偏移来更新位置
    void rel_update(float ang_offset);

private:
    PIDParams *param_;

    uint32_t current_impulse_;
    float target_angle_;

    const uint8_t gear_ratio_;
    const uint16_t pulses_per_round_;
    const uint16_t delta_;

    TIM_HandleTypeDef *pwm_htim_;
    Encoder *encoder_;
    uint32_t pwm_channel_;

    GPIO *ain1_;  // TB6612的AIN1引脚，控制电机正向转动
    GPIO *ain2_;  // TB6612的AIN2引脚，控制电机反向转动

    // @brief 执行并调整电机指向方位
    void move(uint32_t pwm_value);
    // @brief 停止转动
    void stop();
    // @brief 执行电机的移动操作
    void execute();
    // @brief 计算从脉冲次数到输出圈数的转换。这一步会考虑减速比
    float impulse_to_angle(uint32_t impulse) const;
    // @brief 从输出端的角度转换为脉冲次数
    uint32_t angle_to_impulse(float angle) const;
    // @brief 更新PID的参数。这个成员函数每隔DELTA微秒都要调用一次
    float pid_update();
};
