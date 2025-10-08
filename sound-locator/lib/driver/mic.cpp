#include <cstdint>
#include <memory>
#include <stm32f4xx.h>
#include "filter/filter.hpp"
#include "driver/mic.hpp"

constexpr uint8_t MIC_NUMBER = 4;
constexpr uint16_t FREQUENCY = 1000;
constexpr uint16_t MEAN = 2048;
constexpr float TRIGGER_TRHESH = 3096.0f;

// @brief 当前测量的工作模式
enum class Mode: uint8_t{
    Single,  // 单次定位，正弦波
    Continuous,  // 连续追踪，正弦波
    Measure,  // 测量距离，正弦波
    Random  // 播放最炫民族风，确定声源位置，单次
};


struct WaveformSnapshot{
    std::array<float, MIC_NUMBER> filtered;
    uint32_t timestamp;
};

class Microphone{
    public:
        Microphone(uint32_t adc_channel)
            : adc_channel_(adc_channel),
              triggered_(false),
              trig_time_(std::nullopt),
              reach_zero_(std::nullopt),
              last_value_(MEAN)
            {
                filter_ = std::make_unique<KalmanFilter>();
            }
        
        // @brief 用来更新数值，这个数值会经过滤波器过滤
        void update(uint16_t raw_adc, uint32_t timestamp){
            float current = static_cast<float>(raw_adc);
            float filtered = filter_ -> update(current);

            if (last_value_ * current < 0.0f){
                reach_zero_ = timestamp;
            }
        }

        // @brief 重置一个麦克风
        void reset(){
            triggered_ = false;
            trig_time_ = std::nullopt;
            last_value_ = 2048.0f;
            reach_zero_ = std::nullopt;
        }

        // @brief 当前的麦克风是否被触发了。这个成员函数只适用于TDoA模式
        bool is_triggered() const{
            return triggered_;
        }

    private:
        float last_value_;
        std::unique_ptr<KalmanFilter> filter_;
        bool triggered_;  // 麦克风是否接收到了声音
        std::optional<uint32_t> trig_time_;  // TDoA模式下，计算首次触发的时间戳
        std::optional<uint32_t> reach_zero_;  // 上一次穿过均值的时间
        uint32_t adc_channel_;
};

/*
 * @warning 这个麦克风阵列并没有校验是否进行了初始化（start）。
 * 请务必先调用start函数，不然这个阵列无法发挥正常的作用
 */
class MicrophoneMatrix{
    public:
        MicrophoneMatrix(
            ADC_HandleTypeDef* hadc, 
            TIM_HandleTypeDef* timer
        )
        : hadc_(hadc), timer_(timer){
            using std::array, std::unique_ptr;
            microphones_ = array<unique_ptr<Microphone>, MIC_NUMBER> {0};
            buffer_ = array<uint32_t, MIC_NUMBER> {0};
            is_ready_ = false;
            mode_ = Mode::Single;
        }

        // @brief 麦克风阵列，启动！
        void start(){
            HAL_TIM_Base_Start(timer_);
            HAL_ADC_Start_DMA(hadc_, buffer_.data(), 4);
        }
        
        // @bug 如果ADC采样次数过高，CPU来不及处理，缓冲区会不会溢出？
        void callback(){
            uint32_t timestamp = __HAL_TIM_GET_COUNTER(timer_);
            for (int i = 0; i < MIC_NUMBER; i++){
                microphones_[i] -> update(buffer_[i], timestamp);
            }

            check_if_ready_();
        }

        // @brief 改变麦克风的工作模式
        void change_mode(){
            uint8_t curr_val = static_cast<uint8_t>(mode_);
            mode_ = static_cast<Mode>((curr_val + 1) % 3);

            for (const auto& mic : microphones_){
                mic -> reset();
            }
        }

        bool is_ready() const {
            return is_ready_;
        }

    private:
        ADC_HandleTypeDef* hadc_;
        TIM_HandleTypeDef* timer_;
        std::array<std::unique_ptr<Microphone>, MIC_NUMBER> microphones_;
        std::array<uint32_t, MIC_NUMBER> buffer_;
        Mode mode_;
        bool is_ready_;

        // @details 这个成员函数只在TDoA的模式下可用
        void check_if_ready_(){
            if (mode_ == Mode::Single || mode_ == Mode::Random){
                if (is_ready_) return;
            }

            uint8_t count = 0;
            for (const auto& mic : microphones_){
                if (mic -> is_triggered()) count ++;
            }
            if (count >= 3) {
                is_ready_ = true;
            }
        }
};
