/*
 * 使用TDOA的思路来进行二维定位，使用Chan氏解算算法来还原位置
 */
#include <cstdint>
#include "utils.h"

constexpr float VOICE_SPEED = 3.4e-2f;  // 单位：cm/us

using Point = std::pair<float, float>;
using Vector = std::array<float, 2>; // 进行Chan氏反解算通常只需要两个二维向量相乘
using Matrix = std::array<float, 4>; // 扁平化的2x2矩阵，**是C样式的**，先填充横行，再填充纵列

/*
 * @brief 从传入的三个坐标位置和三个时间戳，反算出声源的位置
 * 反解算过程基于Chan氏反解算算法完成，详情参阅：
 * https://zhuanlan.zhihu.com/p/11782284967
 *
 * @details 声速的单位是cm/us，时间戳的单位是us（微秒）
 * 
 * @param locations `array<Point, 3>` 三个麦克风的坐标
 * @param timestamps `array<uint32_t, 3>` 三个麦克风记录到声音的时间戳
 */
Point ComputePosition(
    std::array<Point, 3> locations,
    std::array<uint32_t, 3> timestamps)
{
    // 感觉很多步骤是可以向量化加速的，但是时间紧迫先放一放
    auto [c1, c2, c3] = locations;
    auto [x1, y1] = c1;
    auto [x2, y2] = c2;
    auto [x3, y3] = c3;
    auto [t1, t2, t3] = timestamps;
    // @warning 在长时间运行的过程中，从uint32_t转换到int32_t可能会发生溢出和预期外行为。这个时间约为71.58分钟
    float dr1 = VOICE_SPEED * static_cast<float>(static_cast<int32_t>(t2) - static_cast<int32_t>(t1));
    float dr2 = VOICE_SPEED * static_cast<float>(static_cast<int32_t>(t3) - static_cast<int32_t>(t1));

    float d1 = Utils::GetDistance({0, 0}, c1);
    float d2 = Utils::GetDistance({0, 0}, c2);
    float d3 = Utils::GetDistance({0, 0}, c3);

    float l1 = (d2 * d2 - d1 * d1 - dr1 * dr1) / 2;
    float l2 = (d3 * d3 - d1 * d1 - dr2 * dr2) / 2;

    Matrix matrixA = std::array<float, 4>{x2 - x1, y2 - y1, x3 - x1, y3 - y1};
    Vector vecDr = std::array<float, 2>{-dr1, -dr2};
    Vector vecD = std::array<float, 2>{l1, l2};

    Matrix transposeA = Utils::TransposeMatrix(matrixA);
    Matrix inverted = Utils::InvertMatrix(
        Utils::MultiplyMatrix(transposeA, matrixA)
    );
    Matrix temp = Utils::MultiplyMatrix(inverted, transposeA);
    Vector a = Utils::MultiplyMatrixAndVector(temp, vecDr);
    Vector b = Utils::MultiplyMatrixAndVector(temp, vecD);
    
    float I = a[0] * a[0] + a[1] * a[1] - 1;
    float J = a[0] * (b[0] - x1) + a[1] * (b[1] - y1);
    float K = (x1 - b[0]) * (x1 - b[0]) + (y1 - b[1]) * (y1 - b[1]);

    float sqrt_result;
    arm_sqrt_f32(J * J - I * K, &sqrt_result);
    float r = -(J + sqrt_result)/I;

    float x = a[0] * r + b[0];
    float y = a[1] * r + b[1];

    return {x, y};
}