#include "roboteq_ros2_driver/roboteq_configuration.hpp"

namespace roboteq_ros2_driver
{
namespace configuration
{

std::vector<RequiredControllerSetting> required_controller_settings(
  bool open_loop,
  int encoder_ppr,
  double max_amps,
  int max_rpm)
{
  const int motor_mode = open_loop ? 0 : 1;
  const int amp_limit = static_cast<int>(max_amps * 10);

  return {
    {"ECHOF", 0, 1},
    {"RWD", 0, 1000},
    {"MMOD", 1, motor_mode},
    {"MMOD", 2, motor_mode},
    {"ALIM", 1, amp_limit},
    {"ALIM", 2, amp_limit},
    {"MXRPM", 1, max_rpm},
    {"MXRPM", 2, max_rpm},
    {"MAC", 1, 20000},
    {"MAC", 2, 20000},
    {"MDEC", 1, 20000},
    {"MDEC", 2, 20000},
    {"KP", 1, 1},
    {"KP", 2, 1},
    {"KI", 1, 7},
    {"KI", 2, 7},
    {"KD", 1, 0},
    {"KD", 2, 0},
    {"EPPR", 1, encoder_ppr},
    {"EPPR", 2, encoder_ppr},
  };
}

}  // namespace configuration
}  // namespace roboteq_ros2_driver
