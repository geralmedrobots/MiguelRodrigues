#ifndef ROBOTEQ_ROS2_DRIVER__DRIVER_PARAMETER_VALIDATION_HPP_
#define ROBOTEQ_ROS2_DRIVER__DRIVER_PARAMETER_VALIDATION_HPP_

#include <optional>
#include <functional>
#include <string>

namespace roboteq_ros2_driver
{
namespace parameter_validation
{

struct DriverParameters
{
  std::string port;
  int baud{0};
  double wheel_radius{0.0};
  double wheelbase{0.0};
  int encoder_ppr{0};
  int encoder_cpr{0};
  int encoder_eppr{0};
  int motor_sign_1{0};
  int motor_sign_2{0};
  int encoder_sign_1{0};
  int encoder_sign_2{0};
  int command_angular_sign{0};
  double max_amps{0.0};
  int max_rpm{0};
  double command_timeout_s{0.0};
  int serial_read_timeout_ms{0};
  int serial_write_timeout_ms{0};
  int serial_transaction_timeout_ms{0};
  int serial_max_response_bytes{0};
  double serial_reconnect_interval_s{0.0};
  int encoder_poll_period_ms{0};
  std::string channel_1;
  std::string channel_2;
};

struct ValidationError
{
  std::string parameter;
  std::string reason;
};

std::optional<ValidationError> validate(const DriverParameters & parameters);

// The callback is the startup boundary. Callers must perform all subsystem and ROS
// entity initialization inside it. It is never invoked for invalid parameters.
std::optional<ValidationError> validate_then_start(
  const DriverParameters & parameters,
  const std::function<void()> & start_serial_subsystem);

}  // namespace parameter_validation
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__DRIVER_PARAMETER_VALIDATION_HPP_
