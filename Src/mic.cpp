#include "mic.hpp"

MicrophoneMatrix::MicrophoneMatrix() : mic_buffer_{}, timestamp_records_{},
    last_value_({2048, 2048, 2048, 2048})
{
    for (int i = 0; i < 4; i++)
    {
        filters_[i] = std::make_unique<ZLEMAFilter>(FILTER_PERIOD);
    }
}

bool MicrophoneMatrix::is_ok() const
{
    switch (mode_)
    {
    case WorkingMode::Single:
    case WorkingMode::Measure:
    case WorkingMode::Random:
    {
        if (!last_direction_.has_value())
        {
            return false;
        }

        const bool use_up = last_direction_.value();
        for (const auto &record : timestamp_records_)
        {
            const auto &sample = use_up ? record.first : record.second;
            if (!sample.has_value())
            {
                return false;
            }
        }
        return true;
    }
    case WorkingMode::Continuous:
    {
        auto count = [&](bool up)
        {
            uint8_t c = 0;
            for (const auto &r : timestamp_records_)
            {
                if (up ? r.first.has_value() : r.second.has_value())
                {
                    c++; // there should be rust++
                }
            }
            return c;
        };

        return ((count(true) >= 3) || (count(false) >= 3));
    }
    }
    return false;
}

void MicrophoneMatrix::switch_mode()

{
    uint8_t current = static_cast<uint8_t>(mode_);
    current = (current + 1) % 4;
    mode_ = static_cast<WorkingMode>(current);

    gate_open_time_.reset();
    gate_direction_.reset();
    last_direction_.reset();
}

void MicrophoneMatrix::update(uint32_t timestamp)
{
    for (int i = 0; i < 4; i++)
    {
        float filtered = filters_[i]->update(mic_buffer_[i]);
        uint32_t curr = static_cast<uint32_t>(filtered);
        uint32_t last = last_value_[i];

        const auto UP_T = TRIGGER_THRESH + BUFFER;
        const auto DOWN_T = TRIGGER_THRESH - BUFFER;

        bool up = (last < DOWN_T) && (curr >= UP_T); // 是否是向上穿过判定先
        bool down = (last > UP_T) && (curr <= DOWN_T);

        if (up || down)
        { // 启动接收窗口
            if (!gate_direction_.has_value())
            {
                gate_open_time_ = timestamp;
                gate_direction_ = up;
                last_direction_.reset();

                for (auto &r : timestamp_records_)
                {
                    r = {std::nullopt, std::nullopt};
                }
            }

            if (gate_direction_.has_value() && gate_open_time_.has_value() && gate_direction_.value() == up)
            {
                uint32_t dt = timestamp - gate_open_time_.value();
                if (dt < WINDOW)
                {
                    if (up)
                    {
                        timestamp_records_[i].first = timestamp;
                    }
                    else
                    {
                        timestamp_records_[i].second = timestamp;
                    }
                }
            }
        }

        last_value_[i] = curr;
    }

    if (gate_open_time_.has_value())
    {
        auto dt = timestamp - gate_open_time_.value();

        if (dt > WINDOW)
        {
            auto count_dir = [&](bool is_up)
            {
                uint8_t c = 0;
                for (auto &r : timestamp_records_)
                {
                    if (is_up ? r.first.has_value() : r.second.has_value())
                    {
                        c++;
                    }
                }
                return c;
            };

            auto span_dir = [&](bool is_up)
            {
                uint32_t tmin = U32M, tmax = 0;
                for (auto &r : timestamp_records_)
                {
                    auto opt = is_up ? r.first : r.second;
                    if (opt.has_value())
                    {
                        using namespace std;
                        auto v = opt.value();
                        tmin = min(tmin, v);
                        tmax = max(tmax, v);
                    }
                }
                return (tmin == U32M) ? U32M : (tmax - tmin);
            };

            const bool up_ok = (count_dir(true) >= 3);
            const bool down_ok = (count_dir(false) >= 3);

            if (up_ok || down_ok)
            {
                bool selected_direction = gate_direction_.value_or(true);
                if (up_ok && down_ok)
                {
                    selected_direction = (span_dir(true) <= span_dir(false));
                }
                else
                {
                    selected_direction = up_ok;
                }

                gate_direction_ = selected_direction;
                last_direction_ = selected_direction;
            }

            if (dt > GUARANTEE)
            {
                gate_direction_.reset();
                gate_open_time_.reset();
            }
        }
    }
}

// 这个函数不会校验当前麦克风阵列是否准备好了
// 在调用的时候自行调用is_ok()检查
std::array<uint32_t, 4> MicrophoneMatrix::get_timestamps()
{
    std::array<uint32_t, 4> ts{};
    ts.fill(U32M);

    switch (mode_)
    {
    case WorkingMode::Single:
    case WorkingMode::Measure:
    case WorkingMode::Random:
    {
        const bool use_up = last_direction_.value_or(true);
        for (size_t i = 0; i < ts.size(); ++i)
        {
            const auto &record = timestamp_records_[i];
            ts[i] = use_up ? record.first.value_or(U32M) : record.second.value_or(U32M);
        }
        return ts;
    }
    case WorkingMode::Continuous:
    {
        auto count = [&](bool up) {
            uint8_t c = 0;
            for (auto &r : timestamp_records_)
            {
                if (up ? r.first.has_value() : r.second.has_value())
                {
                    c++;
                }
            }
            return c;
        };

        bool choose_up = count(true) > count(false);

        for (size_t i = 0; i < ts.size(); ++i)
        {
            auto p = timestamp_records_[i];
            ts[i] = choose_up ? p.first.value_or(U32M) : p.second.value_or(U32M);
        }

        return ts;
    }
    }

    return ts;
}
