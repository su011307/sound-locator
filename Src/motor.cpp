#include "motor.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

float wrap_degrees(float angle) {
    while (angle > 180.f) angle -= 360.0f;
    while (angle <= -180.f) angle += 360.0f;
    return angle;
}

Encoder::Encoder(TIM_HandleTypeDef *htim) : htim_(htim)
{
    reset();
}

uint32_t Encoder::get_pulse_count() const
{
    return __HAL_TIM_GET_COUNTER(htim_);
}

void Encoder::reset() const {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

Motor::Motor(
    TIM_HandleTypeDef *pwm_htim,
    uint32_t pwm_channel,
    GPIO *ain1,
    GPIO *ain2,
    Encoder *encoder,
    PIDParams *param)
    : is_moving_(false),
      target_angle_(0.0f),
      gear_ratio_(GEAR_RATIO),
      pulses_per_round_(PULSES_PER_ROUND),
      delta_(DELTA),
      pwm_htim_(pwm_htim),
      encoder_(encoder),
      pwm_channel_(pwm_channel),
      ain1_(ain1),
      ain2_(ain2),
      param_(param)
{
    param->integral = 0.0f;
    param_->last_error = 0.0f;
    encoder_->reset();
    curr_ang_ = wrap_degrees(curr_ang_);
    target_angle_ = wrap_degrees(target_angle_);
    stop();
}

void Motor::stop() {
    ain1_->set_low();
    ain2_->set_low();
    __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, 0);
    HAL_TIM_PWM_Stop(pwm_htim_, pwm_channel_);

    if (encoder_ != nullptr)
    {
        last_pulse_cnt_ = static_cast<uint16_t>(encoder_->get_pulse_count());
    }

    if (param_ != nullptr)
    {
        param_->integral = 0.0f;
        param_->last_error = 0.0f;
    }

    is_moving_ = false;
}

bool Motor::is_busy() const
{
    return is_moving_;
}

void Motor::move_to(const float abs_angle)
{
    target_angle_ = wrap_degrees(abs_angle);
    if (encoder_ != nullptr && !is_moving_)
    {
        last_pulse_cnt_ = static_cast<uint16_t>(encoder_->get_pulse_count());
    }
    is_moving_ = true;
}

void Motor::poll()
{
    if (!is_moving_)
    {
        return;
    }

    if (encoder_ == nullptr || param_ == nullptr)
    {
        return;
    }

    const auto current_cnt = static_cast<uint16_t>(encoder_->get_pulse_count());
    const auto imp_delta = static_cast<int16_t>(current_cnt - last_pulse_cnt_);
    last_pulse_cnt_ = current_cnt;
    const float ang_delta = static_cast<float>(imp_delta) * 360.0f / (pulses_per_round_ * gear_ratio_);

    curr_ang_ += ang_delta;
    curr_ang_ = wrap_degrees(curr_ang_);

    const float error = wrap_degrees(target_angle_ - curr_ang_);

    if (std::abs(error) < STOP_THRESHOLD_ANGLE)
    {
        stop();
        return;
    }

    const float dt = static_cast<float>(delta_) / 1.0e6f;

    param_->integral += error * dt;
    param_->integral = std::clamp(param_->integral, -static_cast<float>(MAX_PWM), static_cast<float>(MAX_PWM));

    const float d = (error - param_->last_error) / dt;
    param_->last_error = error;

    const float output = param_->Kp * error + param_->Ki * param_->integral + param_->Kd * d;
    const bool forward = output > 0.0f;

    const auto pwm_value = static_cast<uint32_t>(std::min(std::fabs(output), static_cast<float>(MAX_PWM)));

    if (pwm_value < MIN_PWM) {
        set_pwm(0);
        return;
    }

    // HAL_TIM_PWM_Stop(pwm_htim_, pwm_channel_);
    // 暂时不清楚为什么注释掉上面这一行就能正常工作
    set_direction(forward);
    set_pwm(pwm_value);
}

void Motor::set_pwm(const uint32_t pwm_value) const {
    __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value);
    HAL_TIM_PWM_Start(pwm_htim_, pwm_channel_);
}

void Motor::set_direction(const bool forward) const {
    if (forward)
    {
        ain1_->set_low();
        ain2_->set_high();
    }
    else
    {
        ain1_->set_high();
        ain2_->set_low();
    }
}

float Motor::get_angle() const {
    return curr_ang_;
}
