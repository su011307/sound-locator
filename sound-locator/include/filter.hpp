#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include "utils.hpp"

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

class ZLEMAFilter{
    public:
        ZLEMAFilter(size_t period);

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
    KalmanFilter(float p_noise, float m_noise, float init_value);
    /*
     * @brief 用来更新卡尔曼滤波器
     * @details 卡尔曼滤波分为两步，先预测，再修正。详情参阅下文注释
     */
    float update(float value);
    float latest() const;

private:
    float x_best_;  // 当前的最优估计
    float p_best_;  // 当前的不确定度协方差。在金融和工程上，我们通常用方差和协方差来衡量风险、噪声等水平，背后隐含了它们符合正态分布的假设

    const float q_;  // 过程噪声协方差。在这个项目中源于物理建模和实际世界的偏差
    const float r_;  // 测量噪声协方差。在我们的模型中源于电路的底噪

    bool is_ready_ = false;
};