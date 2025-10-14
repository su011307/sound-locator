#include "mic.hpp"

Microphone::Microphone(uint32_t adc_channel)
    : adc_channel_(adc_channel),
      triggered_(false),
      trig_time_(std::nullopt),
      reach_zero_(std::nullopt),
      last_value_(MEAN),
      filter_(KalmanFilter(1.0, 2048.0, 2048.0)){}
        
void Microphone::update(uint16_t raw_adc, uint32_t timestamp){
    float current = static_cast<float>(raw_adc);
    float filtered = filter_.update(current);

    if (last_value_ * current < 0.0f){
        reach_zero_ = timestamp;
    }
}

void Microphone::reset(){
    triggered_ = false;
    trig_time_ = std::nullopt;
    last_value_ = 2048.0f;
    reach_zero_ = std::nullopt;
}

bool Microphone::is_triggered() const{
    return triggered_;
}

MicrophoneMatrix::MicrophoneMatrix(
    ADC_HandleTypeDef *hadc,
    TIM_HandleTypeDef *timer)
    : hadc_(hadc), timer_(timer),
      buffer_(std::array<uint32_t, MIC_NUMBER>{0}),
      is_ready_(false), mode_(Mode::Single),
      microphones_({
        // 在这里改麦克风通道的定义
        Microphone(ADC_CHANNEL_0),
        Microphone(ADC_CHANNEL_1),
        Microphone(ADC_CHANNEL_2),
        Microphone(ADC_CHANNEL_3),
    }){ }

void MicrophoneMatrix::start(){
    HAL_TIM_Base_Start(timer_);
    HAL_ADC_Start_DMA(hadc_, buffer_.data(), 4);
}

void MicrophoneMatrix::callback()
{
    uint32_t timestamp = __HAL_TIM_GET_COUNTER(timer_);
    for (int i = 0; i < MIC_NUMBER; i++){
        microphones_[i].update(buffer_[i], timestamp);
    }

    check_if_ready();
}

void MicrophoneMatrix::change_mode()
{
    uint8_t curr_val = static_cast<uint8_t>(mode_);
    mode_ = static_cast<Mode>((curr_val + 1) % 3);

    for (auto& mic : microphones_){
        mic.reset();
    }
}

bool MicrophoneMatrix::is_ready() const 
{
    return is_ready_;
}

void MicrophoneMatrix::check_if_ready()
{
    if (mode_ == Mode::Single || mode_ == Mode::Random){
        if (is_ready_) return;
    }

    uint8_t count = 0;
    for (const auto& mic : microphones_){
        if (mic.is_triggered()) count ++;
    }
    if (count >= 3) {
        is_ready_ = true;
    }
}

Mode MicrophoneMatrix::get_mode() const {
    return mode_;
}

const std::array<uint32_t, MIC_NUMBER>& MicrophoneMatrix::get_timestamps() const{
    return buffer_;
}