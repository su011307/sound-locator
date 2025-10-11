#include <cstdint>
#include <utility>
#include "utils.hpp"

using Point = std::pair<float, float>;

using std::array;

std::optional<Point> compute_position(array<Point, 4> locations, array<uint32_t, 4> timestamps);