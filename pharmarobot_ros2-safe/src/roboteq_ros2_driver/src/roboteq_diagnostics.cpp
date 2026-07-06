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

#include "roboteq_ros2_driver/roboteq_diagnostics.hpp"

#include <algorithm>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace roboteq_ros2_driver
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

int severityForAge(
  const std::optional<std::chrono::milliseconds> & age,
  const std::chrono::milliseconds & warn_threshold,
  const std::chrono::milliseconds & error_threshold)
{
  if (!age.has_value()) {
    return DiagnosticStatus::WARN;
  }
  if (*age >= error_threshold) {
    return DiagnosticStatus::ERROR;
  }
  if (*age >= warn_threshold) {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::OK;
}

std::string ageField(const std::optional<std::chrono::milliseconds> & age)
{
  if (!age.has_value()) {
    return "age=unknown";
  }
  std::ostringstream stream;
  stream << "age_ms=" << age->count();
  return stream.str();
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
  const std::string & name,
  int level,
  const std::string & message,
  const std::string & reason,
  const std::string & age_field)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.level = level;
  status.message = message;
  status.hardware_id = "roboteq";
  status.values.resize(2);
  status.values[0].key = "reason";
  status.values[0].value = reason;
  status.values[1].key = "age";
  status.values[1].value = age_field;
  return status;
}

void appendValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

std::string workerStateName(SerialConnectionState state)
{
  switch (state) {
    case SerialConnectionState::disconnected: return "disconnected";
    case SerialConnectionState::connecting: return "connecting";
    case SerialConnectionState::configuring: return "configuring";
    case SerialConnectionState::waiting_for_fresh_command: return "waiting_for_fresh_command";
    case SerialConnectionState::ready: return "ready";
    case SerialConnectionState::unhealthy: return "unhealthy";
    case SerialConnectionState::reconnecting: return "reconnecting";
  }
  return "unknown";
}

diagnostic_msgs::msg::DiagnosticStatus makeControllerSafetyStatus(
  const std::string & name,
  ControllerSafetySignal signal)
{
  switch (signal) {
    case ControllerSafetySignal::normal:
      return makeStatus(name, DiagnosticStatus::OK, "normal", "controller reports normal", "n/a");
    case ControllerSafetySignal::active:
      return makeStatus(
        name, DiagnosticStatus::ERROR, "active", "controller reports active unsafe state", "n/a");
    case ControllerSafetySignal::unknown:
      return makeStatus(
        name, DiagnosticStatus::WARN, "unknown", "controller state is unknown", "n/a");
    case ControllerSafetySignal::unavailable:
      return makeStatus(
        name, DiagnosticStatus::WARN, "unavailable", "controller state is unavailable", "n/a");
    case ControllerSafetySignal::unsupported:
      return makeStatus(
        name, DiagnosticStatus::WARN, "unsupported", "controller state is not polled", "n/a");
  }
  return makeStatus(name, DiagnosticStatus::WARN, "unknown", "controller state is unknown", "n/a");
}

int logLevelForStatusLevel(int status_level)
{
  if (status_level >= DiagnosticStatus::ERROR) {
    return DiagnosticStatus::ERROR;
  }
  if (status_level == DiagnosticStatus::WARN) {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::OK;
}

SerialConnectionState connectionState(const DiagnosticsState & state)
{
  if (!state.worker_status.has_value()) {
    return state.serial_connected ? SerialConnectionState::ready :
           SerialConnectionState::disconnected;
  }
  return state.worker_status->connection_state;
}

int serialLevel(const DiagnosticsState & state)
{
  if (!state.serial_connected) {
    return DiagnosticStatus::ERROR;
  }
  if (!state.worker_status.has_value()) {
    return state.serial_ready ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
  }
  if (connectionState(state) == SerialConnectionState::ready) {
    return state.worker_status->framing_state == SerialFramingState::synchronized ?
           DiagnosticStatus::OK : DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::WARN;
}

std::string serialMessage(const DiagnosticsState & state)
{
  if (!state.serial_connected) {
    return "disconnected";
  }
  if (!state.worker_status.has_value()) {
    return state.serial_ready ? "ready" : "connected but not ready";
  }
  if (connectionState(state) == SerialConnectionState::waiting_for_fresh_command) {
    return "waiting for fresh command";
  }
  if (state.worker_status->framing_state == SerialFramingState::unresolved) {
    return "diagnostic framing unresolved";
  }
  return state.serial_ready && connectionState(state) == SerialConnectionState::ready ?
         "ready" : "connected but not ready";
}

std::string serialReason(const DiagnosticsState & state)
{
  if (!state.serial_connected) {
    return "transport closed or reconnecting";
  }
  if (!state.worker_status.has_value()) {
    return state.serial_ready ? "transport open and motion-ready" : "transport open but not ready";
  }
  if (connectionState(state) == SerialConnectionState::waiting_for_fresh_command) {
    return "fresh command required after reconnect";
  }
  if (state.worker_status->framing_state == SerialFramingState::unresolved) {
    return "normal transactions suspended during bounded diagnostic recovery";
  }
  return state.serial_ready && connectionState(state) == SerialConnectionState::ready ?
         "transport open and motion-ready" : "transport open but not ready";
}

int commandLevel(const DiagnosticsState & state, const DiagnosticsConfig & config)
{
  if (state.command_timed_out) {
    return DiagnosticStatus::ERROR;
  }
  if (!state.command_active) {
    return DiagnosticStatus::WARN;
  }
  return severityForAge(
    state.command_age,
    config.command_watchdog_warn,
    config.command_watchdog_error);
}

std::string commandMessage(int level, const DiagnosticsState & state)
{
  if (state.command_timed_out || level == DiagnosticStatus::ERROR) {
    return "command timeout";
  }
  if (!state.command_active) {
    return "waiting for first command";
  }
  return level == DiagnosticStatus::OK ? "command fresh" : "command stale";
}

std::string commandReason(int level, const DiagnosticsState & state)
{
  if (state.command_timed_out || level == DiagnosticStatus::ERROR) {
    return "command age exceeded timeout";
  }
  if (!state.command_active) {
    return "no command received yet";
  }
  return level == DiagnosticStatus::OK ? "command within timeout" :
         "command age approaching timeout";
}

}  // namespace

bool DiagnosticsPublisherState::shouldPublish(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  return evaluate(msg, std::chrono::steady_clock::now(), std::chrono::milliseconds::max()).publish;
}

DiagnosticsPublicationDecision DiagnosticsPublisherState::evaluate(
  const diagnostic_msgs::msg::DiagnosticArray & msg,
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds publish_period)
{
  DiagnosticsPublicationDecision decision;
  const auto fingerprint = diagnosticsFingerprint(msg);
  decision.state_changed = !last_publication_time_.has_value() || fingerprint != last_fingerprint_;
  decision.periodic = last_publication_time_.has_value() &&
    now - *last_publication_time_ >= publish_period;
  decision.publish = decision.state_changed || decision.periodic;
  if (decision.publish) {
    last_fingerprint_ = fingerprint;
    last_publication_time_ = now;
  }
  return decision;
}

diagnostic_msgs::msg::DiagnosticArray buildDiagnosticsArray(
  const rclcpp::Time & stamp,
  const DiagnosticsState & state,
  const DiagnosticsConfig & config)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.reserve(5);

  const auto serial_level = serialLevel(state);
  auto serial_status = makeStatus(
    "roboteq/serial_connection",
    serial_level,
    serialMessage(state),
    serialReason(state),
    "n/a");
  if (state.worker_status.has_value()) {
    appendValue(
      serial_status,
      "connection_state",
      workerStateName(state.worker_status->connection_state));
    appendValue(
      serial_status,
      "connection_generation",
      std::to_string(state.worker_status->connection_generation));
    appendValue(
      serial_status,
      "serial_framing",
      state.worker_status->framing_state == SerialFramingState::synchronized ?
      "synchronized" : "unresolved");
    appendValue(
      serial_status,
      "diagnostic_recovery_pending",
      state.worker_status->diagnostic_recovery_pending ? "true" : "false");
  }
  array.status.push_back(serial_status);

  const auto watchdog_level = commandLevel(state, config);
  array.status.push_back(
    makeStatus(
      "roboteq/command_watchdog",
      watchdog_level,
      commandMessage(watchdog_level, state),
      commandReason(watchdog_level, state),
      ageField(state.command_age)));

  const auto encoder_age = state.encoder_sample_available ? state.encoder_age :
    std::optional<std::chrono::milliseconds>{};
  const auto encoder_level = severityForAge(
    encoder_age,
    config.encoder_freshness_warn,
    config.encoder_freshness_error);
  array.status.push_back(
    makeStatus(
      "roboteq/encoder_freshness",
      encoder_level,
      encoder_age.has_value() ? (encoder_level == DiagnosticStatus::OK ? "fresh" : "stale") :
      "no samples yet",
      encoder_age.has_value() ? "age tracked from latest encoder sample" :
      "no encoder sample available",
      ageField(encoder_age)));

  array.status.push_back(
    makeControllerSafetyStatus(
      "roboteq/controller_faults", state.controller_faults));
  array.status.push_back(
    makeControllerSafetyStatus(
      "roboteq/controller_sto", state.sto_status));

  return array;
}

std::string diagnosticsFingerprint(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  std::ostringstream stream;
  for (const auto & status : msg.status) {
    stream << "|" << status.name << ":" << status.level << ":" << status.message;
    for (const auto & kv : status.values) {
      if (kv.key == "age") {
        continue;
      }
      stream << ":" << kv.key << "=" << kv.value;
    }
  }
  return stream.str();
}

std::vector<DiagnosticsLogRecord> buildDiagnosticsLogRecords(
  const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  std::vector<DiagnosticsLogRecord> records;
  records.reserve(msg.status.size());
  for (const auto & status : msg.status) {
    std::ostringstream stream;
    stream << status.name << ": " << status.message;
    const auto reason = std::find_if(
      status.values.begin(),
      status.values.end(),
      [](const diagnostic_msgs::msg::KeyValue & kv) {
        return kv.key == "reason";
      });
    if (reason != status.values.end()) {
      stream << " (" << reason->value << ")";
    }
    records.push_back(DiagnosticsLogRecord{logLevelForStatusLevel(status.level), stream.str()});
  }
  return records;
}

}  // namespace roboteq_ros2_driver
