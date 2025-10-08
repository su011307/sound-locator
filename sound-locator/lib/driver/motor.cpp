#include "driver/motor.hpp"

constexpr uint8_t GEAR_RATIO = 34;         // 减速比
constexpr uint16_t PULSES_PER_ROUND = 500; // 电机每转一圈产生的脉冲数
constexpr uint16_t DELTA = 100;            // 时间间隔，单位：us
constexpr uint32_t MAX_PWM = 99;           // PWM的最大值

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
    Encoder(TIM_HandleTypeDef *htim, uint16_t delta_t)
        : htim_(htim), 
          delta_t_(delta_t)
    {
        reset();
    }

    // @brief 获取当前总脉冲数
    uint32_t get_pulse_count() const
    {
        return static_cast<uint32_t>(__HAL_TIM_GET_COUNTER(htim_));
    }

    // @brief 重置编码器
    void reset()
    {
        __HAL_TIM_SET_COUNTER(htim_, 0);
    }

private:
    TIM_HandleTypeDef *htim_;
    uint8_t gear_ratio_ = GEAR_RATIO;
    uint16_t delta_t_;
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

    // @brief 获取当前电机指向的角度
    float current_angle() const
    {
        int32_t current_pulses = encoder_ -> get_pulse_count();
        return impulse_to_angle(current_pulses);
    }

    // @brief 按照单位圆内绝对角度更新位置
    void abs_update(float target_ang)
    {
        target_angle_ = target_ang;
    }

    // @brief 按照角度的偏移来更新位置
    void rel_update(float ang_offset)
    {
        float curr_ang = current_angle();
        target_angle_ = curr_ang + ang_offset;
    }

    // @brief 执行
    void execute(){
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
    
private:
    PIDParams *param_;
    
    uint32_t current_impulse_; // 内部采用脉冲的数量来计量，减少浮点运算
    float target_angle_;
    
    const uint8_t gear_ratio_ = GEAR_RATIO;
    const uint16_t pulses_per_round_ = PULSES_PER_ROUND;
    const uint16_t delta = DELTA;  // 离散的时间间隔
    
    TIM_HandleTypeDef *pwm_htim_;
    Encoder *encoder_;
    uint32_t pwm_channel_;
    
    GPIO *ain1_;
    GPIO *ain2_;
    
    void move(uint32_t pwm_value)
    {
        __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value);
    }
    
    void stop()
    {
        ain1_ -> set_low();
        ain2_ -> set_low();
        __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, 0);
    }
    
    // @brief 计算从脉冲次数到输出圈数的转换。这一步会考虑减速比
    float impulse_to_angle(uint32_t impulse) const
    {
        float round_motor = static_cast<float>(impulse) / static_cast<float>(pulses_per_round_);
        float output = round_motor / static_cast<float>(gear_ratio_);
        return output * 360.0f;
    } // 单位：度
    
    // @brief 从输出端的角度转换为脉冲次数
    uint32_t angle_to_impulse(float angle) const
    {
        float output = angle / 360.0f;
        float round_motor = output * static_cast<float>(gear_ratio_);
        float impulse_count = round_motor * static_cast<float>(pulses_per_round_);
        return impulse_count;
    } // 单位：度，取整数

    // @brief 更新PID的参数。这个成员函数每隔DELTA微秒都要调用一次
    float pid_update()
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
};
