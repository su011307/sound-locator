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


class KalmanFilter {
public:
    KalmanFilter(float p_noise, float m_noise, float init_value);

    float update(float value);
    float latest() const;

private:
    float x_best_;
    float p_best_;

    const float q_;
    const float r_;

    bool is_ready_ = false;
};