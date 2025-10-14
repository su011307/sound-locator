#include <cstddef>
#include <vector>
#include <stdexcept>
#include "utils.hpp"

/*
 * [DISCLAIMER]
 * 在这里使用了Google Gemini，来完善错误处理和构造函数
 * @brief 一个环形缓冲区，用来以常数复杂度写入麦克风读取到的数据
 */
class CircleBuffer
{
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
    : buffer_(capacity), capacity_(capacity)
{
    if (capacity == 0)
        throw std::invalid_argument("Invalid capacity");
}

/*
 * @brief 向环形缓冲区中写入一份数据
 * @param value `float` 等待写入的数据
 */
void CircleBuffer::push(float value)
{
    buffer_[head_] = value;
    head_ = (head_ + 1) % capacity_; // 移动head_指针，如果到达末尾，则返回开头，实现事实上的循环
    if (curr_size_ < capacity_)
    {
        curr_size_++;
    }
}

/*
 * @brief 获取 `steps` 个间隔之前的数据
 * @param steps `size_t` 间隔的长度
 * @returns float 取得的数据
 */
float CircleBuffer::get_past(size_t steps) const
{
    if (steps >= curr_size_)
    {
        throw std::out_of_range("Request steps out of range");
    }
    size_t index = (head_ + capacity_ - 1 - steps) % capacity_;

    return buffer_[index];
}

/*
 * 检查缓冲区中已经写入的数据数量
 */
size_t CircleBuffer::size() const
{
    return curr_size_;
}

/*
 * 检查缓冲区的最大容量
 */
size_t CircleBuffer::capacity() const
{
    return capacity_;
}

/*
 * 检查缓冲区是否被写满了
 */
bool CircleBuffer::is_full() const
{
    return curr_size_ == capacity_;
}

/*
 * @brief ZLEMA是一种滤波器，可以用来从噪声（比如电路底噪）中分离出信号
 * @details ZLEMA是一种在金融时序数据上常见的去噪手段，是EMA的改进型。相比于EMA，它
 * 通过施加一个相位抵消掉了延迟。这在测定位置的时候可能会更有用，因为延迟可能会导致精度误差
 * @details 意欲了解ZLEMA详情，请参阅：https://juejin.cn/post/7485259847471497225
 */
class ZLEMAFilter
{
public:
    ZLEMAFilter(size_t period)
        : period_(period),
          lag_((period - 1) / 2),
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

/*
 * @brief 更新ZLEMA滤波器的值
 */
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

/*
 * @brief 标量版本的卡尔曼滤波器
 * @details 鉴于我堪称羸弱的线性代数基础，我决定还是退化到标量吧。
 * 标量版本的卡尔曼滤波也是足够应付我们的项目的
 */
class KalmanFilter {
public:
    /*
     * @param p_noise `const float` 过程噪声的方差（或者你们可能看到资料上说协方差，但是在标量下这俩是一回事）
     * @param m_noise `const float` 测量噪声的方差
     * @param init_value `const float` 初始值
     */
    KalmanFilter(
        const float p_noise,
        const float m_noise,
        const float init_value
        )
        : q_(p_noise), r_(m_noise)
    {
        x_best_ = init_value;
        p_best_ = 1.0f;
    }

    /*
     * @brief 用来更新卡尔曼滤波器
     * @details 卡尔曼滤波分为两步，先预测，再修正。详情参阅下文注释
     */
    float update(const float value) {
        if (!is_ready_) {
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
    };

    float latest() const {
        return x_best_;
    }

private:
    float x_best_;  // 当前的最优估计
    float p_best_;  // 当前的不确定度协方差。在金融和工程上，我们通常用方差和协方差来衡量风险、噪声等水平，背后隐含了它们符合正态分布的假设

    const float q_;  // 过程噪声协方差。在这个项目中源于物理建模和实际世界的偏差
    const float r_;  // 测量噪声协方差。在我们的模型中源于电路的底噪

    bool is_ready_ = false;
};