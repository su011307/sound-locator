#include <cstdint>
#include <memory>
#include <stm32f4xx.h>
#include "filter/filter.hpp"
#include "driver/mic.hpp"

constexpr uint16_t FREQUENCY = 1000;
constexpr float TRIGGER_TRHESH = 2048.0f;

// @brief 当前测量的工作模式
enum class Mode: uint8_t{
    Single,
    Continuous,
    Measure,
    Random
};

class Microphone{
    public:
        void update(uint16_t raw_adc, uint32_t timestamp){
            float current = static_cast<float>(raw_adc);
            float filtered = filter_.update();
        }

        void reset(){

        }

        void change_mode();
    private:
        std::unique_ptr<MicKalmanFilter> filter_;
        uint32_t adc_channel_;
};

class MicrophoneMatrix{
    public:

        // @brief 麦克风阵列，启动！
        void start(){
            HAL_TIM_Base_Start(timer_);
            HAL_ADC_Start_DMA(hadc_, buffer_.data(), 4);
        }
        
        // @bug 如果ADC采样次数过高，CPU来不及处理，缓冲区会不会溢出？
        void callback(){
            uint32_t timestamp = __HAL_TIM_GET_COUNTER(timer_);
            mic0_->update(buffer_[0], timestamp);
            mic1_->update(buffer_[1], timestamp);
            mic2_->update(buffer_[2], timestamp);
            mic3_->update(buffer_[3], timestamp);
        }
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

        void check_is_ready(){

        }
};
