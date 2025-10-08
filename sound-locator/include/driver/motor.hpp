#pragma once
#include <cstdint>
#include <array>
#include <stm32f4xx.h>

struct PIDParams{
    float kp;
    float ki;
    float kd;
};

class GPIO{
public:
    GPIO(GPIO_TypeDef *port, uint16_t pin);
    void set_high();
    void set_low();
    void reverse();
private:
    GPIO_TypeDef *port_;
    uint16_t pin_;
};

class Encoder{
public:
    Encoder(TIM_HandleTypeDef* htim, uint16_t delta_t);
    uint32_t get_pulse_count() const;
    void reset();
private:
    uint8_t gear_ratio_;
    uint16_t delta_t_;
    TIM_HandleTypeDef* htim_;
};

class Motor{
public:
    Motor(
        TIM_HandleTypeDef *pwm_htim,
        uint32_t pwm_channel,
        const GPIO& ain1,
        const GPIO& ain2,
        Encoder& encoder,
        PIDParams& config
    );
    float get_current_angle() const;
    void abs_update(float target_ang);
    void rel_update(float ang_offset);
    void pid_update();
private:
    float ki_;
    float kp_;
    float kd_;
    float int_;
    float err_;
    uint32_t current_impulse_;
    float target_angle_;
    const uint8_t gear_ratio_;
    const uint16_t pulses_per_round_;
    const uint16_t delta;
    TIM_HandleTypeDef* pwm_htim_;
    uint32_t pwm_channel_;
    Encoder& encoder_;
    GPIO ain1_;
    GPIO ain2_;
    void move(uint32_t pwm_value);
    void stop();
    float impulse_to_angle(uint32_t impulse) const;
    uint32_t angle_to_impulse(float angle) const;
};

