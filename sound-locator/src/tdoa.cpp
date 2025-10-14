/*
 * 使用TDOA的思路来进行二维定位，使用Chan氏解算算法来还原位置
 */
#include <cstdint>
#include "utils.hpp"

constexpr float VOICE_SPEED = 3.43e-2f; // 单位：cm/us

using Point = std::pair<float, float>;

/*
 * @brief 从传入的三个坐标位置和三个时间戳，反算出声源的位置
 * 反解算过程基于Chan氏反解算算法完成，详情参阅：
 * https://zhuanlan.zhihu.com/p/11782284967
 *
 * @details 声速的单位是cm/us，时间戳的单位是us（微秒）
 *
 * @param locations `array<Point, 4>` 四个麦克风的坐标
 * @param timestamps `array<uint32_t, 4>` 四个麦克风记录到声音的时间戳
 */
std::optional<Point> compute_position(
    std::array<Point, 4> locations,
    std::array<uint32_t, 4> timestamps)
{
    auto [t1, t2, t3, t4] = timestamps;

    auto [x1, y1] = locations[0];
    auto [x2, y2] = locations[1];
    auto [x3, y3] = locations[2];
    auto [x4, y4] = locations[3];

    float dr1 = VOICE_SPEED * (static_cast<int64_t>(t2) - static_cast<int64_t>(t1));
    float dr2 = VOICE_SPEED * (static_cast<int64_t>(t3) - static_cast<int64_t>(t1));
    float dr3 = VOICE_SPEED * (static_cast<int64_t>(t4) - static_cast<int64_t>(t1));

    float d1, d2, d3, d4;

    arm_sqrt_f32(x1 * x1 + y1 * y1, &d1);
    arm_sqrt_f32(x2 * x2 + y2 * y2, &d2);
    arm_sqrt_f32(x3 * x3 + y3 * y3, &d3);
    arm_sqrt_f32(x4 * x4 + y4 * y4, &d4);

    float l1 = (d2 * d2 - d1 * d1 - dr1 * dr1) / 2;
    float l2 = (d3 * d3 - d1 * d1 - dr2 * dr2) / 2;
    float l3 = (d4 * d4 - d1 * d1 - dr3 * dr3) / 2;

    Matrix A(3, 2, std::vector<float>{x2 - x1, y2 - y1, x3 - x1, y3 - y1, x4 - x1, y4 - y1});
    Matrix Dr(3, 1, std::vector<float>{-dr1, -dr2, -dr3});
    Matrix D(3, 1, std::vector<float>{l1, l2, l3});

    Vector a = inverse(transpose(A).value() * A).value() * transpose(A).value() * Dr;
    Vector b = inverse(transpose(A).value() * A).value() * transpose(A).value() * D;

    float I = a.figures[0] * a.figures[0] + a.figures[1] * a.figures[1] - 1;
    float J = a.figures[0] * (b.figures[0] - x1) + a.figures[1] * (b.figures[1] - y1);
    float K = (x1 - b.figures[0]) * (x1 - b.figures[0]) + (y1 - b.figures[1]) * (y1 - b.figures[1]);

    float sqrt_result, temp = J * J - I * K;
    
    if (temp < 0)
    {
        return std::nullopt;
    }
    arm_sqrt_f32(temp, &sqrt_result);
    float r = -(J + sqrt_result) / I;

    float x = a.figures[0] * r + b.figures[0];
    float y = a.figures[1] * r + b.figures[1];

    return Point{x, y};
}