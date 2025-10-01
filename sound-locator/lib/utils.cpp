#include "utils.h"

namespace Utils{
    float GetDistance(const Point& begin, const Point& end){
        auto [x1, y1] = begin;
        auto [x2, y2] = end;

        float dx = x2 - x1;
        float dy = y2 - y1;

        float distanceSquare = dx * dx + dy * dy;
        float distance;

        arm_sqrt_f32(distanceSquare, &distance);  // 这一步可以用硬件加速平方根计算
        return distance;
    }

    float VectorDot(const Vector& vecA, const Vector& vecB){
        float dotResult;
        arm_dot_prod_f32(vecA.data(), vecB.data(), vecB.size(), &dotResult);  // 和GetDistance同理
        return dotResult;
    }

    Matrix TransposeMatrix(const Matrix& matrix){
        Matrix result;
        arm_matrix_instance_f32 matrixInstance;
        arm_matrix_instance_f32 resultInstance;

        arm_mat_init_f32(&matrixInstance, 2, 2, (float32_t*)matrix.data());
        arm_mat_init_f32(&resultInstance, 2, 2, result.data());

        arm_mat_trans_f32(&matrixInstance, &resultInstance);
        return result;
    }

    Matrix InvertMatrix(const Matrix& matrix){
        Matrix result;
        
        arm_matrix_instance_f32 matrixInstance;
        arm_matrix_instance_f32 resultInstance;

        arm_mat_init_f32(&matrixInstance, 2, 2, (float32_t*)matrix.data());
        arm_mat_init_f32(&resultInstance, 2, 2, result.data());

        arm_mat_inverse_f32(&matrixInstance, &resultInstance);
        return result;
    }

    Matrix MultiplyMatrix(const Matrix& matA, const Matrix& matB){
        Matrix result;

        arm_matrix_instance_f32 matAInstance;
        arm_matrix_instance_f32 matBInstance;
        arm_matrix_instance_f32 resultInstance;

        arm_mat_init_f32(&matAInstance, 2, 2, (float32_t*)matA.data());
        arm_mat_init_f32(&matBInstance, 2, 2, (float32_t*)matB.data());
        arm_mat_init_f32(&resultInstance, 2, 2, result.data());

        arm_mat_mult_f32(&matAInstance, &matBInstance, &resultInstance);

        return result;
    }

    Vector MultiplyMatrixAndVector(const Matrix& matrix, const Vector& vector){
        Vector result;
        result[0] = matrix[0] * vector[0] + matrix[1] * vector[1];
        result[1] = matrix[2] * vector[0] + matrix[3] * vector[1];
        return result;
    }
}

namespace Filter{
    
}