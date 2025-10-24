#include "motor.hpp"
#include <cmath>

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
    target_angle_ = abs_angle;
    if (encoder_ != nullptr)
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
    if (curr_ang_ >= 360.0f) curr_ang_ -= 360.0f;
    if (curr_ang_ < 0.0f) curr_ang_ += 360.0f;

    const float error = target_angle_ - curr_ang_;

    if (std::abs(error) < STOP_THRESHOLD_ANGLE)
    {
        stop();
        return;
    }

    const float dt = static_cast<float>(delta_) / 1.0e6f;

    param_->integral += error * dt;

    if (param_->integral > static_cast<float>(MAX_PWM)) param_->integral = MAX_PWM;
    if (param_->integral < -static_cast<float>(MAX_PWM)) param_->integral = -static_cast<float>(MAX_PWM);

    const float d = (error - param_->last_error) / dt;
    param_->last_error = error;

    const float output = param_->Kp * error + param_->Ki * param_->integral + param_->Kd * d;
    set_direction(output > 0.0);

    auto pwm_value = static_cast<uint32_t>(std::abs(output));

    pwm_value = pwm_value > MAX_PWM ? MAX_PWM : pwm_value;
    set_pwm(pwm_value);
}

void Motor::set_pwm(const uint32_t pwm_value) const {
    __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value);
    HAL_TIM_PWM_Start(pwm_htim_, pwm_channel_);
}

void Motor::set_direction(const bool forward) const {
    if (forward)
    {
        ain1_->set_high();
        ain2_->set_low();
    }
    else
    {
        ain1_->set_low();
        ain2_->set_high();
    }
}

float Motor::get_angle() const {
    return curr_ang_;
}
