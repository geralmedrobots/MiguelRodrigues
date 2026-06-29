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
//    * Neither the name of the Geralmedrobots nor the names of its
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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_DIAGNOSTICS_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_DIAGNOSTICS_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/time.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

namespace roboteq_ros2_driver
{

struct DiagnosticsConfig
{
  std::chrono::milliseconds publish_period{1000};
  std::chrono::milliseconds encoder_freshness_warn{250};
  std::chrono::milliseconds encoder_freshness_error{1000};
  std::chrono::milliseconds command_watchdog_warn{250};
  std::chrono::milliseconds command_watchdog_error{1000};
};

enum class ControllerSafetySignal
{
  unavailable,
  unsupported,
  unknown,
  normal,
  active,
};

struct DiagnosticsState
{
  bool serial_connected{false};
  bool serial_ready{false};
  bool command_active{false};
  bool command_timed_out{false};
  bool encoder_sample_available{false};
  std::optional<std::chrono::milliseconds> command_age{};
  std::optional<std::chrono::milliseconds> encoder_age{};
  std::optional<SerialWorkerStatus> worker_status{};
  ControllerSafetySignal controller_faults{ControllerSafetySignal::unsupported};
  ControllerSafetySignal sto_status{ControllerSafetySignal::unsupported};
};

struct DiagnosticsPublicationDecision
{
  bool publish{false};
  bool state_changed{false};
  bool periodic{false};
};

struct DiagnosticsLogRecord
{
  int level{0};
  std::string message;
};

class DiagnosticsPublisherState
{
public:
  bool shouldPublish(const diagnostic_msgs::msg::DiagnosticArray & msg);
  DiagnosticsPublicationDecision evaluate(
    const diagnostic_msgs::msg::DiagnosticArray & msg,
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds publish_period);

private:
  std::string last_fingerprint_;
  std::optional<std::chrono::steady_clock::time_point> last_publication_time_;
};

diagnostic_msgs::msg::DiagnosticArray buildDiagnosticsArray(
  const rclcpp::Time & stamp,
  const DiagnosticsState & state,
  const DiagnosticsConfig & config);

std::string diagnosticsFingerprint(const diagnostic_msgs::msg::DiagnosticArray & msg);

std::vector<DiagnosticsLogRecord> buildDiagnosticsLogRecords(
  const diagnostic_msgs::msg::DiagnosticArray & msg);

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_DIAGNOSTICS_HPP_
