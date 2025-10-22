#include <cstddef>
#include <vector>
#include <stdexcept>
#include "utils.hpp"
#include "filter.hpp"

CircleBuffer::CircleBuffer(size_t capacity)
    : buffer_(capacity), capacity_(capacity),
      head_(0), curr_size_(0)
{
    if (capacity == 0)
    {
        // Error_Handler();
    } 
}

void CircleBuffer::push(float value)
{
    buffer_[head_] = value;
    head_ = (head_ + 1) % capacity_; // 移动head_指针，如果到达末尾，则返回开头，实现事实上的循环
    if (curr_size_ < capacity_)
    {
        curr_size_++;
    }
}

float CircleBuffer::get_past(size_t steps) const
{
    if (steps >= curr_size_)
    {
        // Error_Handler();
    }
    size_t index = (head_ + capacity_ - 1 - steps) % capacity_;

    return buffer_[index];
}

std::vector<float> CircleBuffer::content() const
{
    std::vector<float> result;
    if (curr_size_ == 0)
    {
        return result;
    }

    result.reserve(curr_size_);

    const size_t start = (head_ + capacity_ - curr_size_) % capacity_;

    if (start + curr_size_ > capacity_)
    {
        result.insert(result.end(), buffer_.begin() + start, buffer_.end());
        result.insert(result.end(), buffer_.begin(), buffer_.begin() + head_);
    }
    else
    {
        result.insert(result.end(), buffer_.begin() + start, buffer_.begin() + start + curr_size_);
    }

    return result;
}

size_t CircleBuffer::size() const
{
    return curr_size_;
}

size_t CircleBuffer::capacity() const
{
    return capacity_;
}

bool CircleBuffer::is_full() const
{
    return curr_size_ == capacity_;
}

ZLEMAFilter::ZLEMAFilter(size_t period)
    : period_(period),
        lag_((period - 1) / 2),
        alpha_(2.0f / (period + 1.0f)),
        history_(lag_ + 1)
{
    is_ready_ = false;
    prev_zlema_ = 0.0f;
}

float ZLEMAFilter::update(float value)
{
    history_.push(value);
    if (!is_ready_)
    {
        if (history_.is_full())
        {
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

KalmanFilter::KalmanFilter(
    const float p_noise,
    const float m_noise,
    const float init_value)
    : q_(p_noise), r_(m_noise)
{
    x_best_ = init_value;
    p_best_ = 1.0f;
}

float KalmanFilter::update(const float value)
{
    if (!is_ready_)
    {
        x_best_ = value;
        is_ready_ = true;
        return x_best_;
    }

    // 预测步，用上一步的最优不确定度+过程噪声来先验估计当前的不确定度
    float p_prd = p_best_ + q_;
    // 计算卡尔曼收益。卡尔曼收益的大小决定了我们倾向于相信预测还是测量
    // 其实这一步很好定性理解，不确定度和测量噪声，谁的占比越低越倾向于相信谁
    float k_gain = p_prd / (p_prd + r_);
    // 使用实际的测量结果来修正先验预测值。卡尔曼增益决定了修正权重
    // 有点像机器学习中的RL，模型先预测，在根据实际情况修正
    x_best_ = x_best_ + k_gain * (value - x_best_);
    p_best_ = (1.0f - k_gain) * p_prd;

    return x_best_;
}

float KalmanFilter::latest() const
{
    return x_best_;
}