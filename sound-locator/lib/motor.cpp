/*
 * @brief 这个文件实现一个电机的抽象
 * @details Motor的控制采用了大名鼎鼎的PID算法。意欲了解PID，请参阅：
 * 
 */

#pragma once
#include <cstdint>
#include <array>
#include <stm32f4xx.h>

constexpr uint8_t GEAR_RATIO = 34;  // 减速比
constexpr uint16_t PULSES_PER_ROUND = 500;  // 电机每转一圈产生的脉冲数
constexpr uint16_t DELTA = 100;  // 时间间隔，单位：us

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

GPIO::GPIO(GPIO_TypeDef *port, uint16_t pin)
    : port_(port), pin_(pin) {}

void GPIO::set_high(){
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

void GPIO::set_low(){
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

void GPIO::reverse(){
    HAL_GPIO_TogglePin(port_, pin_);
}


struct PIDParams{
    float kp;
    float ki;
    float kd;
};

class Encoder{
    // @bug 可能没有正确处理计时器初始化的问题
    public:
        Encoder(TIM_HandleTypeDef* htim, uint16_t delta_t)
            : htim_(htim), delta_t_(delta_t){
                reset();
            }

        // @brief 获取当前总脉冲数
        uint32_t get_pulse_count() const{
            return static_cast<uint32_t>(__HAL_TIM_GET_COUNTER(htim_));
        }

        // @brief 重置编码器
        void reset(){
            __HAL_TIM_SET_COUNTER(htim_, 0);
        }
    private:
        uint8_t gear_ratio_ = GEAR_RATIO;
        uint16_t delta_t_;
        TIM_HandleTypeDef* htim_;
};

// [DISCLAMER] 在构建电机抽象的时候使用了Qwen查询并解释电机的性能参数
// 并查阅了有关的HAL API的使用细节
class Motor{
    public:
        Motor(
            TIM_HandleTypeDef *pwm_htim,
            uint32_t pwm_channel,
            const GPIO& ain1,
            const GPIO& ain2,
            Encoder& encoder,
            PIDParams& config
        )
        : pwm_htim_(pwm_htim), pwm_channel_(pwm_channel),
          ain1_(ain1), ain2_(ain2), encoder_(encoder),
          target_angle_(0.0f), current_impulse_(0), ki_(config.ki),
          kp_(config.kp), err_(0.0f)
        {
            stop();
        }

        // @brief 获取当前电机指向的角度
        float get_current_angle() const{
            int32_t current_pulses = encoder_.get_pulse_count();
            return impulse_to_angle(current_pulses);
        }

        // @brief 按照单位圆内绝对角度更新位置
        void abs_update(float target_ang){
            target_angle_ = target_ang;
        }

        // @brief 按照角度的偏移来更新位置
        void rel_update(float ang_offset){
            float curr_ang = get_current_angle();
            target_angle_ = curr_ang + ang_offset;
        }
        
        // @brief 更新PID的参数。这个成员函数每隔DELTA微秒都要调用一次
        void pid_update(){
            float curr_ang = get_current_angle();
            float err = target_angle_ - curr_ang;

            // DELTA的单位是微秒，转换为秒。* 1.0是常用的技巧，用来隐式转换类型，因为static_cast<T>(var)写起来太麻烦
            int_ += err * ((delta * 1.0) / (1e6 * 1.0));

            float derivative = (err - err_) / (delta * 1.0 / (1e6 * 1.0));
            float output = kp_ * err + ki_ * int_ + kd_ * derivative;
        }


    private:
        float ki_;
        float kp_;
        float kd_;

        float int_;  // PID中的积分项
        float err_;  // PID中的误差项

        uint32_t current_impulse_;  // 内部采用脉冲的数量来计量，减少浮点运算
        float target_angle_;

        const uint8_t gear_ratio_ = GEAR_RATIO;
        const uint16_t pulses_per_round_ = PULSES_PER_ROUND;
        const uint16_t delta = DELTA;

        TIM_HandleTypeDef* pwm_htim_;
        uint32_t pwm_channel_;
        Encoder& encoder_;
        GPIO ain1_;
        GPIO ain2_;

        void move(uint32_t pwm_value){
            __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, pwm_value);
        }

        void stop(){
            ain1_.set_low();
            ain2_.set_low();
            __HAL_TIM_SET_COMPARE(pwm_htim_, pwm_channel_, 0);
        }
        
        // @brief 计算从脉冲次数到输出圈数的转换。这一步会考虑减速比
        float impulse_to_angle(uint32_t impulse) const{
            float round_motor = static_cast<float>(impulse) / static_cast<float>(pulses_per_round_);
            float output = round_motor / static_cast<float>(gear_ratio_);
            return output * 360.0f;
        }  // 单位：度

        // @brief 从输出端的角度转换为脉冲次数
        uint32_t angle_to_impulse(float angle) const{
            float output = angle / 360.0f;
            float round_motor = output * static_cast<float>(gear_ratio_);
            float impulse_count = round_motor * static_cast<float>(pulses_per_round_);
            return impulse_count;
        }  // 单位：度，取整数
};
