#ifndef ROBOTEQ_ROS2_DRIVER__ODOM_TF_HPP_
#define ROBOTEQ_ROS2_DRIVER__ODOM_TF_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace roboteq_ros2_driver
{
namespace odom_tf
{

geometry_msgs::msg::TransformStamped build_odom_to_base_transform(
  const std::string & odom_frame,
  const std::string & base_frame,
  const builtin_interfaces::msg::Time & stamp,
  double x,
  double y,
  double yaw);

}  // namespace odom_tf
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ODOM_TF_HPP_
