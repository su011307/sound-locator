#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <optional>
#include <stdexcept>
#include <utility>

#include "arm_math.h"

/*
 * @brief 这个类是矩阵的抽象，实现了常见的加法和乘法运算
 * @details 对于两个矩阵，可以直接使用“+”和“*”运算符来进行运算，因为这些运算符被“重载”了
 * 你可以理解为被重新定义了
 */
class Matrix
{
public:
    Matrix(
        const uint16_t rows,
        const uint16_t columns,
        std::optional<std::vector<float>> figure_data = std::nullopt);

    uint16_t rows;
    uint16_t columns;
    std::vector<float> figures; // C样式的扁平化矩阵，以行为单位

    Matrix operator+(const Matrix &other) const;
    Matrix operator*(const Matrix &other) const;
};

using Point = std::pair<float, float>;
using Vector = Matrix;

float get_distance(const Point &begin, const Point &end);
std::optional<Matrix> transpose(const Matrix &matrix);
std::optional<Matrix> inverse(const Matrix &matrix);
