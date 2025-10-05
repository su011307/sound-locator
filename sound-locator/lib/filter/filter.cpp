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
 * @details ZLEMA是一种在金融时序数据上常见的去噪手段，是EMA的改进型。相比于Kalman，它
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
 * 更新ZLEMA滤波器的值
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
 * [DISCLAIMER]
 * 在学习和实现线性卡尔曼滤波的时候使用了Gemini
 *
 * @brief 适用于麦克风的卡尔曼滤波器
 * @details 金融上很常用，但我从来没有自己实现过一个滤波器
 * 所以在做完整的测试之前，使用这个滤波器的时候应当保持谨慎
 * 它有可能会出现难以察觉的错误
 */
class MicKalmanFilter
{
public:
    float update(float value)
    {
        x_predict_ = predict_state_(&x_best_, &A_);
        P_ = predict_covariance_(&A_, &Q_, &P_);
        Matrix H_trans = transpose(H_).value();
        Matrix S = H_ * P_ * H_trans + R_;

        Matrix K_gain = P_ * H_trans * inverse(S).value();

        Vector predicted_measurement = matrix_vector_multiply(H_, x_predict_).value();

        float residual_y = value - predicted_measurement[0];
        return 0.0f;
    };
    float latest_value() const;

private:
    Vector x_best_;    // 最优状态
    Vector x_predict_; // 预测状态
    Matrix P_;         // 协方差矩阵: 衡量不确定度的大小
    Matrix A_;         // 状态转移矩阵: 从上一时刻最优状态线性外推到此时刻预测状态
    Matrix H_;         // 测量矩阵: 将状态空间映射到测量空间
    Matrix R_;         // 测量噪声协方差矩阵: 衡量传感器本身的不确定度
    Matrix Q_;         // 过程噪声协方差矩阵:

    /*
     * @brief 计算卡尔曼增益矩阵
     * @details 卡尔曼增益矩阵的值决定系统倾向于采纳预测值或者观测值
     * 使用指针而不是引用是为了避免多次在栈上分配空间降低性能
     * @param
     */
    Matrix kalman_gain_(const Matrix *H, const Matrix *P, const Matrix *R)
    {
        return (*P) * transpose((*H)).value() * inverse((*H) * (*P) * transpose((*H)).value() + (*R)).value();
    }

    Vector predict_state_(const Vector *x_best, const Matrix *A)
    {
        return matrix_vector_multiply((*A), (*x_best)).value();
    }

    Matrix predict_covariance_(const Matrix *A, const Matrix *Q, const Matrix *P)
    {
        return (*A) * (*P) * transpose(*A).value() + (*Q);
    }
};