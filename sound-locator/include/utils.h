#include <array>
#include <utility>
#include "arm_math.h"

/*
* @brief 这里面定义了很多TDoA所需要的工具函数
* @details 所有的函数都使用了arm_math.h库的函数进行硬件加速计算
* 为保证团队协作效率和安全，尽量使用了现代C++的特性和风格
*/
namespace Utils{
    using Point = std::pair<float, float>;
    using Vector = std::array<float, 2>;  // 进行Chan氏反解算通常只需要两个二维向量相乘
    using Matrix = std::array<float, 4>;  // 扁平化的2x2矩阵，**是C样式的**，先填充横行，再填充纵列

    /*
    * @brief 计算两个点之间的直线距离，使用朴素的勾股定理
    * @param begin `Point` 起始点的坐标
    * @param end `Point` 终点的坐标
    * @return float 两个点之间的直线距离
    */
    float GetDistance(const Point& begin, const Point& end);

    /*
    * @brief 计算两个二维向量的点乘
    * @param vecA `Vector` 第一个向量{x1, y1}
    * @param vecB `Vector` 第二个向量{x2, y2}
    * @return float 向量点乘的结果
    */
    float VectorDot(const Vector& vecA, const Vector& vecB);

    /*
     * @brief 转置一个矩阵
     * @param matrix `const Matrix&` 需要转置的矩阵引用
     * @return `Matrix` 转置完成的矩阵
     */
    Matrix TransposeMatrix(const Matrix& matrix);
    
    /*
    * @brief 计算一个矩阵的逆
    * @param matrix `const Matrix&` 需要计算逆的矩阵引用
    * @return `Matrix` 矩阵的逆
    */
    Matrix InvertMatrix(const Matrix& matrix);

    /*
    * @brief 将两个二维矩阵相乘
    * @param matA `const Matrix&` 第一个矩阵的引用
    * @param matB `const Matrix&` 第二个矩阵的引用
    * @return `Matrix` 相乘之后的矩阵结果
    */
    Matrix MultiplyMatrix(const Matrix& matA, const Matrix& matB);

    /*
     * @brief 将一个矩阵和一个二维向量相乘
     * @param matrix `const Matrix&` 矩阵的引用
     * @param vector `const Vector&` 向量的引用
     * @return `Vector` 相乘之后的矩阵结果
     */
    Vector MultiplyMatrixAndVector(const Matrix& matrix, const Vector& vector);
}