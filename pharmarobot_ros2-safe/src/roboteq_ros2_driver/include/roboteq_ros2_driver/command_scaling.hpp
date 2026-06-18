#ifndef ROBOTEQ_ROS2_DRIVER__COMMAND_SCALING_HPP_
#define ROBOTEQ_ROS2_DRIVER__COMMAND_SCALING_HPP_

namespace roboteq_ros2_driver
{
namespace command_scaling
{

struct CommandPair
{
  double first;
  double second;
};

CommandPair scale_pair_to_limit(double first, double second, double limit);

}  // namespace command_scaling
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__COMMAND_SCALING_HPP_
