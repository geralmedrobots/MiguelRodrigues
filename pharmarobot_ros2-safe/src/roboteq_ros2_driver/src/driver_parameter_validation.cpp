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

#include "roboteq_ros2_driver/driver_parameter_validation.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace roboteq_ros2_driver
{
namespace parameter_validation
{
namespace
{

std::optional<ValidationError> positive_finite(const char * name, double value)
{
  if (!std::isfinite(value)) {
    return ValidationError{name, "must be finite"};
  }
  if (value <= 0.0) {
    return ValidationError{name, "must be positive"};
  }
  return std::nullopt;
}

std::optional<ValidationError> positive_int(const char * name, int value)
{
  if (value <= 0) {
    return ValidationError{name, "must be positive"};
  }
  return std::nullopt;
}

std::optional<ValidationError> explicit_sign(const char * name, int value)
{
  if (value != -1 && value != 1) {
    return ValidationError{name, "must be exactly -1 or 1"};
  }
  return std::nullopt;
}

std::optional<ValidationError> nonzero_magnitude(const char * name, int value)
{
  if (value == 0) {
    return ValidationError{name, "magnitude must be non-zero"};
  }
  if (value == std::numeric_limits<int>::min()) {
    return ValidationError{name, "magnitude exceeds the representable signed integer range"};
  }
  return std::nullopt;
}

std::optional<ValidationError> positive_magnitude(const char * name, int value)
{
  if (value <= 0) {
    return ValidationError{name, "must be positive"};
  }
  return std::nullopt;
}

bool is_channel_name(const std::string & value)
{
  return value == "left" || value == "right";
}

}  // namespace

std::optional<ValidationError> validate_encoder_freshness_thresholds(
  double warn_s, double error_s)
{
  if (const auto error = positive_finite("encoder_freshness_warn_s", warn_s)) {
    return error;
  }
  if (const auto error = positive_finite("encoder_freshness_error_s", error_s)) {
    return error;
  }
  if (error_s <= warn_s) {
    return ValidationError{
      "encoder_freshness_error_s", "must be greater than encoder_freshness_warn_s"};
  }
  const double max_seconds_for_milliseconds =
    static_cast<double>(std::numeric_limits<int>::max()) / 1000.0;
  if (warn_s > max_seconds_for_milliseconds) {
    return ValidationError{
      "encoder_freshness_warn_s", "exceeds the software conversion limit for milliseconds"};
  }
  if (error_s > max_seconds_for_milliseconds) {
    return ValidationError{
      "encoder_freshness_error_s", "exceeds the software conversion limit for milliseconds"};
  }
  return std::nullopt;
}

std::optional<ValidationError> validate(const DriverParameters & p)
{
  if (p.port.empty()) {
    return ValidationError{"port", "must not be empty"};
  }
  if (const auto error = positive_int("baud", p.baud)) {
    return error;
  }
  if (const auto error = positive_finite("wheel_radius", p.wheel_radius)) {
    return error;
  }
  if (const auto error = positive_finite("wheelbase", p.wheelbase)) {
    return error;
  }
  if (const auto error = positive_magnitude("encoder_ppr", p.encoder_ppr)) {
    return error;
  }
  if (const auto error = positive_magnitude("encoder_cpr", p.encoder_cpr)) {
    return error;
  }
  if (const auto error = nonzero_magnitude("encoder_eppr", p.encoder_eppr)) {
    return error;
  }
  if (const auto error = explicit_sign("motor_sign_1", p.motor_sign_1)) {
    return error;
  }
  if (const auto error = explicit_sign("motor_sign_2", p.motor_sign_2)) {
    return error;
  }
  if (const auto error = explicit_sign("encoder_sign_1", p.encoder_sign_1)) {
    return error;
  }
  if (const auto error = explicit_sign("encoder_sign_2", p.encoder_sign_2)) {
    return error;
  }
  if (const auto error = explicit_sign("command_angular_sign", p.command_angular_sign)) {
    return error;
  }
  if (const auto error = positive_finite("max_amps", p.max_amps)) {
    return error;
  }
  const double max_amps_for_tenths =
    static_cast<double>(std::numeric_limits<int>::max()) / 10.0;
  if (p.max_amps > max_amps_for_tenths) {
    return ValidationError{
      "max_amps", "exceeds the software conversion limit for controller amp tenths"};
  }
  if (const auto error = positive_int("max_rpm", p.max_rpm)) {
    return error;
  }
  if (const auto error = positive_finite("cmd_timeout_s", p.command_timeout_s)) {
    return error;
  }
  const double max_seconds_for_milliseconds =
    static_cast<double>(std::numeric_limits<int>::max()) / 1000.0;
  if (p.command_timeout_s > max_seconds_for_milliseconds) {
    return ValidationError{
      "cmd_timeout_s", "exceeds the software conversion limit for milliseconds"};
  }
  if (const auto error = positive_int("serial_read_timeout_ms", p.serial_read_timeout_ms)) {
    return error;
  }
  if (const auto error = positive_int("serial_write_timeout_ms", p.serial_write_timeout_ms)) {
    return error;
  }
  if (const auto error = positive_int(
      "serial_transaction_timeout_ms", p.serial_transaction_timeout_ms))
  {
    return error;
  }
  if (const auto error = positive_int(
      "serial_max_response_bytes", p.serial_max_response_bytes))
  {
    return error;
  }
  if (const auto error = positive_finite(
      "serial_reconnect_interval_s", p.serial_reconnect_interval_s))
  {
    return error;
  }
  if (p.serial_reconnect_interval_s > max_seconds_for_milliseconds) {
    return ValidationError{
      "serial_reconnect_interval_s",
      "exceeds the software conversion limit for milliseconds"};
  }
  if (const auto error = positive_int("encoder_poll_period_ms", p.encoder_poll_period_ms)) {
    return error;
  }
  if (const auto error = positive_finite(
      "diagnostics_publish_rate_hz", p.diagnostics_publish_rate_hz))
  {
    return error;
  }
  if (const auto error = validate_encoder_freshness_thresholds(
      p.encoder_freshness_warn_s, p.encoder_freshness_error_s))
  {
    return error;
  }
  if (!is_channel_name(p.channel_1)) {
    return ValidationError{"channel_1", "must be exactly 'left' or 'right'"};
  }
  if (!is_channel_name(p.channel_2)) {
    return ValidationError{"channel_2", "must be exactly 'left' or 'right'"};
  }
  if (p.channel_1 == p.channel_2) {
    return ValidationError{"channel_2", "must map to a different wheel than channel_1"};
  }
  return std::nullopt;
}

std::optional<ValidationError> validate_then_start(
  const DriverParameters & parameters,
  const std::function<void()> & start_serial_subsystem)
{
  if (const auto error = validate(parameters)) {
    return error;
  }
  start_serial_subsystem();
  return std::nullopt;
}

}  // namespace parameter_validation
}  // namespace roboteq_ros2_driver
