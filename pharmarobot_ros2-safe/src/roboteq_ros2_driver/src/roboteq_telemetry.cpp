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

#include "roboteq_ros2_driver/roboteq_telemetry.hpp"

#include <charconv>
#include <system_error>

namespace roboteq_ros2_driver
{

const std::array<MotorTelemetryQuery, 13> & motorTelemetryQueries()
{
  static const std::array<MotorTelemetryQuery, 13> queries{{
      {MotorTelemetryField::fault_flags, 0, "?FF\r", "FF="},
      {MotorTelemetryField::command_source, 1, "?CIS 1\r", "CIS="},
      {MotorTelemetryField::applied_command, 1, "?M 1\r", "M="},
      {MotorTelemetryField::measured_speed, 1, "?S 1\r", "S="},
      {MotorTelemetryField::current, 1, "?A 1\r", "A="},
      {MotorTelemetryField::power, 1, "?P 1\r", "P="},
      {MotorTelemetryField::motor_fault, 1, "?FM 1\r", "FM="},
      {MotorTelemetryField::command_source, 2, "?CIS 2\r", "CIS="},
      {MotorTelemetryField::applied_command, 2, "?M 2\r", "M="},
      {MotorTelemetryField::measured_speed, 2, "?S 2\r", "S="},
      {MotorTelemetryField::current, 2, "?A 2\r", "A="},
      {MotorTelemetryField::power, 2, "?P 2\r", "P="},
      {MotorTelemetryField::motor_fault, 2, "?FM 2\r", "FM="},
  }};
  return queries;
}

std::optional<int64_t> parseMotorTelemetryInteger(
  std::string_view response,
  std::string_view expected_prefix,
  std::string & error)
{
  error.clear();
  if (response.size() <= expected_prefix.size() ||
    response.substr(0, expected_prefix.size()) != expected_prefix)
  {
    error = "unexpected telemetry response prefix";
    return std::nullopt;
  }
  const auto payload = response.substr(expected_prefix.size());
  int64_t value = 0;
  const auto parsed = std::from_chars(payload.data(), payload.data() + payload.size(), value);
  if (parsed.ec == std::errc::invalid_argument) {
    error = "telemetry value is not an integer";
    return std::nullopt;
  }
  if (parsed.ec == std::errc::result_out_of_range) {
    error = "telemetry integer is out of range";
    return std::nullopt;
  }
  if (parsed.ptr != payload.data() + payload.size()) {
    error = "telemetry value has trailing characters";
    return std::nullopt;
  }
  return value;
}

const char * motorTelemetryFieldName(MotorTelemetryField field)
{
  switch (field) {
    case MotorTelemetryField::command_source: return "CIS";
    case MotorTelemetryField::applied_command: return "M";
    case MotorTelemetryField::measured_speed: return "S";
    case MotorTelemetryField::current: return "A";
    case MotorTelemetryField::power: return "P";
    case MotorTelemetryField::motor_fault: return "FM";
    case MotorTelemetryField::fault_flags: return "FF";
  }
  return "unknown";
}

}  // namespace roboteq_ros2_driver
