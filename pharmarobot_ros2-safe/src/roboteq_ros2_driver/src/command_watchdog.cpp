#include "roboteq_ros2_driver/command_watchdog.hpp"

#include <cmath>

namespace roboteq_ros2_driver
{
namespace command_watchdog
{

bool should_send_timeout_stop(
  bool received_first_command,
  bool timeout_already_logged,
  double command_age_s,
  double timeout_s)
{
  return received_first_command && !timeout_already_logged &&
         std::isfinite(command_age_s) && command_age_s > timeout_s;
}

}  // namespace command_watchdog
}  // namespace roboteq_ros2_driver
