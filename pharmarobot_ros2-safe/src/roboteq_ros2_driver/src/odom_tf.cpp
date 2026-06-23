#include "roboteq_ros2_driver/odom_tf.hpp"

#include <tf2/LinearMath/Quaternion.h>

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
  double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame;
  transform.child_frame_id = base_frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

}  // namespace odom_tf
}  // namespace roboteq_ros2_driver
