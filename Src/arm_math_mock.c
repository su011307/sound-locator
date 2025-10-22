// test/mocks/arm_math_mock.c

#include "arm_math.h"
#include <math.h>   // 用于 sqrtf 和 fabs
#include <stdlib.h> // 包含一些基础功能

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float32_t *pData)
{
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData = pData;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    // 使用C风格的强制类型转换
    uint32_t size = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    uint32_t i;
    for (i = 0; i < size; ++i)
    {
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    uint16_t rowsA = pSrcA->numRows;
    uint16_t colsA = pSrcA->numCols;
    uint16_t colsB = pSrcB->numCols;
    uint16_t i, j, k;

    for (i = 0; i < rowsA; i++)
    {
        for (j = 0; j < colsB; j++)
        {
            float sum = 0.0f;
            for (k = 0; k < colsA; k++)
            {
                sum += pSrcA->pData[i * colsA + k] * pSrcB->pData[k * colsB + j];
            }
            pDst->pData[i * colsB + j] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    uint16_t rows = pSrc->numRows;
    uint16_t cols = pSrc->numCols;
    uint16_t i, j;

    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            pDst->pData[j * rows + i] = pSrc->pData[i * cols + j];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    if (pSrc->numRows != pSrc->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    if (pSrc->numRows == 2)
    { // 专门为2x2矩阵实现
        float a = pSrc->pData[0], b = pSrc->pData[1];
        float c = pSrc->pData[2], d = pSrc->pData[3];

        float det = a * d - b * c;
        // 使用 fabsf 进行浮点数绝对值比较
        if (fabsf(det) < 1e-9f)
        {
            return ARM_MATH_SINGULAR;
        }

        float inv_det = 1.0f / det;
        pDst->pData[0] = d * inv_det;
        pDst->pData[1] = -b * inv_det;
        pDst->pData[2] = -c * inv_det;
        pDst->pData[3] = a * inv_det;
        return ARM_MATH_SUCCESS;
    }

    return ARM_MATH_TEST_FAILURE;
}

float arm_euclidean_distance_f32(const float32_t *pA, const float32_t *pB, uint32_t blockSize)
{
    float sum = 0.0f;
    uint32_t i;
    for (i = 0; i < blockSize; ++i)
    {
        float diff = pA[i] - pB[i];
        sum += diff * diff;
    }
    return sqrtf(sum);
}

void arm_sqrt_f32(float32_t in, float32_t *pOut)
{
    *pOut = sqrtf(in);
}