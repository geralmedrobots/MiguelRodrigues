#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_CONFIGURATION_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_CONFIGURATION_HPP_

#include <string>
#include <vector>

namespace roboteq_ros2_driver
{
namespace configuration
{

struct RequiredControllerSetting
{
  std::string name;
  int channel{0};
  int expected_value{0};
};

std::vector<RequiredControllerSetting> required_controller_settings(
  bool open_loop,
  int encoder_eppr,
  double max_amps,
  int max_rpm);

}  // namespace configuration
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_CONFIGURATION_HPP_
