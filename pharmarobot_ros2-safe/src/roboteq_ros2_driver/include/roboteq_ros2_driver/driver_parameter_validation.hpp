// Copyright 2026 Medrobots
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

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
  std::string odom_frame;
  std::string base_frame;
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
  double diagnostics_publish_rate_hz{0.0};
  std::string channel_1;
  std::string channel_2;
  double encoder_freshness_warn_s{0.25};
  double encoder_freshness_error_s{1.0};
  bool telemetry_enabled{false};
  int telemetry_poll_period_ms{200};
  int telemetry_query_timeout_ms{50};
  int telemetry_stale_after_ms{1000};
};

struct ValidationError
{
  std::string parameter;
  std::string reason;
};

std::optional<ValidationError> validate(const DriverParameters & parameters);

std::optional<ValidationError> validate_encoder_freshness_thresholds(
  double warn_s, double error_s);

std::optional<ValidationError> validate_telemetry_timing(
  bool enabled,
  int poll_period_ms,
  int query_timeout_ms,
  int stale_after_ms,
  int serial_transaction_timeout_ms,
  int command_timeout_ms);

// The callback is the startup boundary. Callers must perform all subsystem and ROS
// entity initialization inside it. It is never invoked for invalid parameters.
std::optional<ValidationError> validate_then_start(
  const DriverParameters & parameters,
  const std::function<void()> & start_serial_subsystem);

}  // namespace parameter_validation
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__DRIVER_PARAMETER_VALIDATION_HPP_
