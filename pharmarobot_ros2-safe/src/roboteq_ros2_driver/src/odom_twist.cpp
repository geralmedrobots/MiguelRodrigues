#include "roboteq_ros2_driver/odom_twist.hpp"

#include <cmath>

namespace roboteq_ros2_driver
{
namespace odom_twist
{

MeasuredTwist calculate_measured_twist(
  double linear_delta,
  double previous_yaw,
  double current_yaw,
  double dt,
  bool has_previous_sample)
{
  if (!has_previous_sample || !std::isfinite(linear_delta) || !std::isfinite(previous_yaw) ||
    !std::isfinite(current_yaw) || !std::isfinite(dt) || dt <= 0.0)
  {
    return {0.0, 0.0};
  }

  const double yaw_delta = std::atan2(
    std::sin(current_yaw - previous_yaw),
    std::cos(current_yaw - previous_yaw));

  return {linear_delta / dt, yaw_delta / dt};
}

}  // namespace odom_twist
}  // namespace roboteq_ros2_driver
