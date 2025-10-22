/*
 * [DISCLAIMER] 在重写基础设施的时候，使用Gemini重新梳理了思路
 * @brief 这个文件提供了在Chan氏反解算、线性卡尔曼滤波中的一些基础
 */
#include "utils.hpp"

Matrix::Matrix(
    const uint16_t rows,
    const uint16_t columns,
    std::optional<std::vector<float>> figure_data)
    : rows(rows), columns(columns), figures(rows * columns, 0.0f)
{
    if (figure_data.has_value())
    {
        const auto &data = figure_data.value();

        // 检查矩阵数据的数量和大小是否对得上
        if (data.size() != static_cast<uint32_t>(rows) * columns)
        {
            // Error_Handler();
        }

        figures = data;
    }
}

// @brief 重载 + 运算符为矩阵的相加运算
Matrix Matrix::operator+(const Matrix &other) const
{
    if (rows != other.rows || columns != other.columns)
    {
        // Error_Handler();
    }

    Matrix result(rows, columns);

    // 使用FPU加速的库函数之前，必须先声明几个`arm_matrix_instance_*`，用来存放计算用的数据和结果
    arm_matrix_instance_f32 self, other_mat, result_mat;
    arm_mat_init_f32(&self, rows, columns, const_cast<float *>(figures.data()));
    arm_mat_init_f32(&other_mat, rows, columns, const_cast<float *>(other.figures.data()));
    arm_mat_init_f32(&result_mat, rows, columns, result.figures.data());

    // 判断计算是否成功
    if (arm_mat_add_f32(&self, &other_mat, &result_mat) != ARM_MATH_SUCCESS)
    {
        // Error_Handler();
    }
    else
    {
        return result;
    }
}

// @brief 重载 * 运算符为矩阵的乘法运算
Matrix Matrix::operator*(const Matrix &other) const
{
    if (columns != other.rows)
    {
        // Error_Handler();
    }
    Matrix result(rows, other.columns);
    arm_matrix_instance_f32 this_inst, other_inst, result_inst;
    arm_mat_init_f32(&this_inst, rows, columns, const_cast<float *>(figures.data()));
    arm_mat_init_f32(&other_inst, other.rows, other.columns, const_cast<float *>(other.figures.data()));
    arm_mat_init_f32(&result_inst, rows, other.columns, result.figures.data());

    if (arm_mat_mult_f32(&this_inst, &other_inst, &result_inst) != ARM_MATH_SUCCESS)
    {
        // Error_Handler();
    }
    else
    {
        return result;
    }
}

// @brief 计算两个点之间的欧几里得距离
float get_distance(const Point &begin, const Point &end)
{
    using namespace std;
    array<float, 2> begin_array = {begin.first, begin.second};
    array<float, 2> end_array = {end.first, end.second};
    float distance = arm_euclidean_distance_f32(begin_array.data(), end_array.data(), 2);
    return distance;
}

/*
 * @brief 对一个任意的矩阵求其转置矩阵
 * @details std::optional<类型>是一个现代C++的特性，代表这个函数
 * 可能返回指定类型的变量，也可能不返回
 */
std::optional<Matrix> transpose(const Matrix &matrix)
{
    // 我承认使用std::optional<T>来自于我用Rust的思维惯性……
    Matrix result(matrix.columns, matrix.rows);

    arm_matrix_instance_f32 raw, result_mat;
    arm_mat_init_f32(&raw, matrix.rows, matrix.columns, const_cast<float *>(matrix.figures.data()));
    arm_mat_init_f32(&result_mat, matrix.columns, matrix.rows, result.figures.data());

    if (arm_mat_trans_f32(&raw, &result_mat) != ARM_MATH_SUCCESS)
    {
        // `std::nullopt` 表示“没有返回值”
        return std::nullopt;
    }
    else
    {
        return result;
    }
}

// @brief 对矩阵求逆
std::optional<Matrix> inverse(const Matrix &matrix)
{
    if (matrix.rows != matrix.columns)
        return std::nullopt;
    Matrix result(matrix.rows, matrix.rows);

    arm_matrix_instance_f32 raw, result_mat;
    arm_mat_init_f32(&raw, matrix.rows, matrix.columns, const_cast<float *>(matrix.figures.data()));
    arm_mat_init_f32(&result_mat, matrix.columns, matrix.rows, result.figures.data());

    if (arm_mat_inverse_f32(&raw, &result_mat) != ARM_MATH_SUCCESS)
    {
        return std::nullopt;
    }
    else
    {
        return result;
    }
}
