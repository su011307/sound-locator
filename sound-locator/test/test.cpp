// test/test_utils.cpp

#include <unity.h>
#include "utils.hpp" // 包含 Matrix, Point 等
#include "tdoa.hpp"  // 直接包含 tdoa.cpp 以便测试其静态函数
#include <math.h>    // 引入 sqrtf 和 powf 用于计算

// Unity 框架的样板代码
void setUp(void) {}
void tearDown(void) {}

// --- 原有的测试用例 ---

void test_matrix_addition(void)
{
    Matrix A(2, 2, std::vector<float>{1.0, 2.0, 3.0, 4.0});
    Matrix B(2, 2, std::vector<float>{5.0, 6.0, 7.0, 8.0});
    Matrix C = A + B;
    TEST_ASSERT_EQUAL_FLOAT(6.0, C.figures[0]);
    TEST_ASSERT_EQUAL_FLOAT(8.0, C.figures[1]);
    TEST_ASSERT_EQUAL_FLOAT(10.0, C.figures[2]);
    TEST_ASSERT_EQUAL_FLOAT(12.0, C.figures[3]);
}

void test_get_distance_function(void)
{
    Point p1 = {0.0, 3.0};
    Point p2 = {4.0, 0.0};
    float expected_distance = 5.0;
    float actual_distance = get_distance(p1, p2);
    TEST_ASSERT_EQUAL_FLOAT(expected_distance, actual_distance);
}

// --- 为 tdoa.cpp 新增的测试用例 ---

void test_tdoa_compute_position(void)
{
    // 1. 设定已知条件 (Ground Truth)
    constexpr float VOICE_SPEED_TEST = 3.43e-2f;  // cm/us
    const Point source_position = {15.5f, 25.2f}; // 假设这是我们想定位的真实声源位置
    const std::array<Point, 4> mic_locations = {
        Point{0.0f, 0.0f},
        Point{9.5f, 0.0f},
        Point{0.0f, 9.5f},
        Point{9.5f, 9.5f}};

    // 2. 正向计算出完美的浮点数时间戳
    float dist1 = get_distance(source_position, mic_locations[0]);
    float dist2 = get_distance(source_position, mic_locations[1]);
    float dist3 = get_distance(source_position, mic_locations[2]);
    float dist4 = get_distance(source_position, mic_locations[3]);

    // 为了避免时间戳为负，我们加一个固定的时间偏移量
    const float time_offset = 1000.0f;
    const std::array<float, 4> perfect_timestamps_float = {
        (dist1 / VOICE_SPEED_TEST) + time_offset,
        (dist2 / VOICE_SPEED_TEST) + time_offset,
        (dist3 / VOICE_SPEED_TEST) + time_offset,
        (dist4 / VOICE_SPEED_TEST) + time_offset,
    };

    // 3. 模拟精度损失：将 float 转换为 uint32_t
    const std::array<uint32_t, 4> timestamps_with_error = {
        (uint32_t)perfect_timestamps_float[0],
        (uint32_t)perfect_timestamps_float[1],
        (uint32_t)perfect_timestamps_float[2],
        (uint32_t)perfect_timestamps_float[3],
    };

    // 4. 调用待测函数，传入带有误差的数据
    std::optional<Point> result = compute_position(mic_locations, timestamps_with_error);

    TEST_ASSERT_TRUE(result.has_value());

    Point calculated_position = result.value();

    // 5. 验证结果：检查计算出的位置是否在真实位置的误差允许范围内
    // 由于时间戳取整引入了误差，我们不能期望结果完全相等
    // TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)
    // delta: 允许的误差范围
    // expected: 期望的真实值
    // actual: 函数计算出的实际值
    const float allowable_error = 0.5f; // 允许 0.5 cm 的误差
    TEST_ASSERT_FLOAT_WITHIN(allowable_error, source_position.first, calculated_position.first);
    TEST_ASSERT_FLOAT_WITHIN(allowable_error, source_position.second, calculated_position.second);
}

// --- 程序主入口 ---
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // 运行所有测试用例
    RUN_TEST(test_matrix_addition);
    RUN_TEST(test_get_distance_function);
    RUN_TEST(test_tdoa_compute_position); // <-- 添加新的测试

    return UNITY_END();
}