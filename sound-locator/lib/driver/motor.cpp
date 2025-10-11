#include "driver/motor.hpp"

constexpr uint8_t GEAR_RATIO = 34;         // 减速比
constexpr uint16_t PULSES_PER_ROUND = 500; // 电机每转一圈产生的脉冲数
constexpr uint16_t DELTA = 100;            // 时间间隔，单位：us
constexpr uint32_t MAX_PWM = 99;           // PWM的最大值

Encoder::Encoder(TIM_HandleTypeDef *htim, uint16_t delta_t)
    : htim_(htim),
      delta_t_(delta_t)
{
    reset();
}

uint32_t Encoder::get_pulse_count() const
{
    return static_cast<uint32_t>(__HAL_TIM_GET_COUNTER(htim_));
}

void Encoder::reset()
{
    __HAL_TIM_SET_COUNTER(htim_, 0);
}

Motor::Motor(
        TIM_HandleTypeDef *pwm_htim,
        uint32_t pwm_channel,
        GPIO *ain1,
        GPIO *ain2,
        Encoder *encoder,
        PIDParams *param
    )
        : pwm_htim_(pwm_htim), 
          pwm_channel_(pwm_channel),
          ain1_(ain1), 
          ain2_(ain2), 
          encoder_(encoder),
          target_angle_(0.0f), 
          current_impulse_(0), 
          param_(param)
    {
        stop();
    }

float Motor::current_angle() const {
    int32_t current_pulses = encoder_ -> get_pulse_count();
    return impulse_to_angle(current_pulses);
}

void Motor::abs_update(float target_ang)
{
    target_angle_ = target_ang;
}

void Motor::rel_update(float ang_offset)
{
    float curr_ang = current_angle();
    target_angle_ = curr_ang + ang_offset;
}

void Motor::execute(){
    float pid_out = pid_update();

    if (pid_out > 0) {
        ain1_ -> set_high();
        ain2_ -> set_low();
    } else {
        ain1_ -> set_low();
        ain2_ -> set_high();
    }

    float abs_out = std::abs(pid_out);

    // 防止PWM值的溢出
    uint32_t pwm_value  = abs_out > MAX_PWM ? MAX_PWM : static_cast<uint32_t>(abs_out);

    move(pwm_value);
}
    

void Motor::move(uint32_t pwm_value)
{
    __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value);
}

void Motor::stop()
{
    ain1_ -> set_low();
    ain2_ -> set_low();
    __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, 0);
}

float Motor::impulse_to_angle(uint32_t impulse) const
{
    float round_motor = static_cast<float>(impulse) / static_cast<float>(pulses_per_round_);
    float output = round_motor / static_cast<float>(gear_ratio_);
    return output * 360.0f;
} // 单位：度

uint32_t Motor::angle_to_impulse(float angle) const
{
    float output = angle / 360.0f;
    float round_motor = output * static_cast<float>(gear_ratio_);
    float impulse_count = round_motor * static_cast<float>(pulses_per_round_);
    return impulse_count;
} // 单位：度，取整数

float Motor::pid_update()
{
    float curr_ang = current_angle();

    float prev_err = param_ -> last_error;
    float err = target_angle_ - curr_ang;
    
    (param_ -> last_error) = err;

    // DELTA的单位是微秒，转换为秒
    // * 1.0是常用的技巧，用来隐式转换类型，因为static_cast<T>(var)写起来太麻烦
    (param_ -> intergral) += err * ((delta * 1.0) / (1e6 * 1.0));

    float derivative = (err - prev_err) / (delta * 1.0 / (1e6 * 1.0));
    float output = (param_ -> Kp) * err + (param_ -> Ki) * (param_ -> intergral) + (param_ -> Kd) * derivative;

    return output;  // 这个output用来通过PWM控制电机，其值应当落在[0, 100]以内
}
