#ifndef ARM_MATH_H_
#define ARM_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef float float32_t;

    typedef enum
    {
        ARM_MATH_SUCCESS = 0,
        ARM_MATH_ARGUMENT_ERROR = -1,
        ARM_MATH_LENGTH_ERROR = -2,
        ARM_MATH_SIZE_MISMATCH = -3,
        ARM_MATH_NANINF = -4,
        ARM_MATH_SINGULAR = -5, // 矩阵为奇异矩阵，不可逆
        ARM_MATH_TEST_FAILURE = -6
    } arm_status;

    typedef struct
    {
        uint16_t numRows;
        uint16_t numCols;
        float32_t *pData;
    } arm_matrix_instance_f32;

    void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float32_t *pData);
    arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
    arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
    arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
    arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
    float arm_euclidean_distance_f32(const float32_t *pA, const float32_t *pB, uint32_t blockSize);
    void arm_sqrt_f32(float32_t in, float32_t *pOut);

#ifdef __cplusplus
}
#endif

#endif // ARM_MATH_H_