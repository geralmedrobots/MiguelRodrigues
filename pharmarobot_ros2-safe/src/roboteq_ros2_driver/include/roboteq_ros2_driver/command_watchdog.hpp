#ifndef ROBOTEQ_ROS2_DRIVER__COMMAND_WATCHDOG_HPP_
#define ROBOTEQ_ROS2_DRIVER__COMMAND_WATCHDOG_HPP_

namespace roboteq_ros2_driver
{
namespace command_watchdog
{

bool should_send_timeout_stop(
  bool received_first_command,
  bool timeout_already_logged,
  double command_age_s,
  double timeout_s);

}  // namespace command_watchdog
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__COMMAND_WATCHDOG_HPP_
