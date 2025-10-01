#include <cstddef>
#include <vector>
#include <stdexcept>

/*
* [DISCLAIMER]
* 在这里使用了Google Gemini，来
* @brief 一个环形缓冲区，用来以常数复杂度写入麦克风读取到的数据
*/
class CircleBuffer{
public:
    explicit CircleBuffer(size_t capacity);
    void push(float value);
    float get_past(size_t steps) const;
    size_t size() const;
    size_t capacity() const;
    bool is_full() const;
private:
    std::vector<float> buffer_;
    size_t head_ = 0;
    size_t curr_size_ = 0;
    const size_t capacity_;
};

CircleBuffer::CircleBuffer(size_t capacity)
    : buffer_(capacity), capacity_(capacity){
        if (capacity == 0) throw std::invalid_argument("Invalid capacity");
    }

void CircleBuffer::push(float value){
    buffer_[head_] = value;
    head_ = (head_ + 1) % capacity_;  // 移动head_指针，如果到达末尾，则返回开头，实现事实上的循环
    if (curr_size_ < capacity_){
        curr_size_ ++;
    }
}

float CircleBuffer::get_past(size_t steps) const{
    if (steps >= curr_size_){
        throw std::out_of_range("Request steps out of range");
    }
    size_t index = (head_ + capacity_ - 1 - steps) % capacity_;

    return buffer_[index];
}

size_t CircleBuffer::size() const {
    return curr_size_;
}

size_t CircleBuffer::capacity() const{
    return capacity_;
}

bool CircleBuffer::is_full() const {
    return curr_size_ == capacity_;
}

/*
* @brief ZLEMA是一种滤波器，可以用来从噪声（比如电路底噪）中分离出信号
* @details ZLEMA是一种在金融时序数据上常见的去噪手段，是EMA的改进型。相比于Kalman，它
* 通过施加一个相位抵消掉了延迟。这在测定位置的时候可能会更有用，因为延迟可能会导致精度误差
*/
class ZLEMAFilter{
public:
    ZLEMAFilter(size_t period)
        : period_(period),
          lag_((period - 1)/2),
          alpha_(2.0f / (period + 1.0f)),
          history_(lag_ + 1)
    {
        is_ready_ = false;
        prev_zlema_ = 0.0f;
    }

    float update(float value);

private:
    size_t period_;
    size_t lag_;
    float alpha_;
    float prev_zlema_;
    bool is_ready_;
    CircleBuffer history_;
};

float ZLEMAFilter::update(float value){
    history_.push(value);
    if (!is_ready_){
        if (history_.is_full()){
            is_ready_ = true;
            prev_zlema_ = history_.get_past(lag_);
        }
        return value;
    }

    float past_value = history_.get_past(lag_);
    float corrected = value + (value - past_value);
    float curr_zlema = alpha_ * corrected + (1.0f - alpha_) * prev_zlema_;

    prev_zlema_ = curr_zlema;
    return curr_zlema;
}