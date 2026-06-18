#ifndef ROBOTEQ_ROS2_DRIVER__ODOM_TWIST_HPP_
#define ROBOTEQ_ROS2_DRIVER__ODOM_TWIST_HPP_

namespace roboteq_ros2_driver
{
namespace odom_twist
{

struct MeasuredTwist
{
  double linear_x;
  double angular_z;
};

MeasuredTwist calculate_measured_twist(
  double linear_delta,
  double previous_yaw,
  double current_yaw,
  double dt,
  bool has_previous_sample);

}  // namespace odom_twist
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ODOM_TWIST_HPP_
