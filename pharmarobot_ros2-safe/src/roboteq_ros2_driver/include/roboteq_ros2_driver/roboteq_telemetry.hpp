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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_TELEMETRY_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_TELEMETRY_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace roboteq_ros2_driver
{

enum class MotorTelemetryField
{
  command_source,
  applied_command,
  measured_speed,
  current,
  power,
  motor_fault,
  fault_flags,
};

struct MotorTelemetryQuery
{
  MotorTelemetryField field;
  int channel;
  const char * command;
  const char * expected_prefix;
};

struct MotorTelemetryChannel
{
  int channel{0};
  int64_t command_source{0};
  int64_t applied_command{0};
  int64_t measured_speed{0};
  int64_t current{0};
  int64_t power{0};
  int64_t motor_fault{0};
  int64_t fault_flags{0};
  std::chrono::steady_clock::time_point timestamp{};
  std::chrono::milliseconds age{0};
  bool valid{false};
  std::string failure_reason{"not sampled"};
};

struct MotorTelemetrySnapshot
{
  MotorTelemetryChannel channel_1{1};
  MotorTelemetryChannel channel_2{2};
  std::chrono::steady_clock::time_point timestamp{};
  std::chrono::milliseconds age{0};
  bool valid{false};
  std::string failure_reason{"not sampled"};
  uint64_t sequence{0};
  uint64_t connection_generation{0};
};

const std::array<MotorTelemetryQuery, 13> & motorTelemetryQueries();

std::optional<int64_t> parseMotorTelemetryInteger(
  std::string_view response,
  std::string_view expected_prefix,
  std::string & error);

const char * motorTelemetryFieldName(MotorTelemetryField field);

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_TELEMETRY_HPP_
