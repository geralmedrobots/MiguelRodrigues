#include "roboteq_ros2_driver/command_scaling.hpp"

#include <algorithm>
#include <cmath>

namespace roboteq_ros2_driver
{
namespace command_scaling
{

CommandPair scale_pair_to_limit(double first, double second, double limit)
{
  if (!std::isfinite(first) || !std::isfinite(second) || !std::isfinite(limit) || limit <= 0.0) {
    return {0.0, 0.0};
  }

  const double max_abs = std::max(std::abs(first), std::abs(second));
  if (max_abs <= limit) {
    return {first, second};
  }

  const double scale = limit / max_abs;
  return {first * scale, second * scale};
}

}  // namespace command_scaling
}  // namespace roboteq_ros2_driver
