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

#ifndef ROBOTEQ_ROS2_DRIVER__PHASE5B_HARNESS_LOGIC_HPP_
#define ROBOTEQ_ROS2_DRIVER__PHASE5B_HARNESS_LOGIC_HPP_

#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

namespace roboteq_ros2_driver
{

struct Phase5bAttemptDecision
{
  bool success{false};
  const char * outcome{"pending"};
};

struct Phase5bDiagnosticEvidence
{
  uint64_t diagnostic_correlation_id{0};
  uint64_t diagnostic_connection_generation{0};
  std::string diagnostic_started_ns;
  std::string diagnostic_write_accepted_ns;
  std::string diagnostic_first_byte_ns;
  std::string diagnostic_last_byte_ns;
  std::string diagnostic_timeout_ns;
  bool diagnostic_delimiter_observed{false};
  std::string diagnostic_byte_count;
  std::string diagnostic_raw;
  std::string diagnostic_raw_hex;
  std::string diagnostic_failure_reason;
};

inline std::string bytesToHex(std::string_view bytes)
{
  static constexpr char kHexDigits[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(bytes.size() * 2);
  for (const unsigned char ch : bytes) {
    hex.push_back(kHexDigits[ch >> 4]);
    hex.push_back(kHexDigits[ch & 0x0f]);
  }
  return hex;
}

inline std::string jsonEscapeExactBytes(std::string_view bytes)
{
  std::string escaped;
  escaped.reserve(bytes.size() * 6);
  for (const unsigned char ch : bytes) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (ch >= 0x20 && ch <= 0x7e) {
          escaped.push_back(static_cast<char>(ch));
        } else {
          std::ostringstream stream;
          stream << "\\u00";
          static constexpr char kHexDigits[] = "0123456789abcdef";
          stream << kHexDigits[ch >> 4] << kHexDigits[ch & 0x0f];
          escaped += stream.str();
        }
        break;
    }
  }
  return escaped;
}

inline std::string time_point_ns_or_null(const std::chrono::steady_clock::time_point & value)
{
  if (value == std::chrono::steady_clock::time_point{}) {
    return "null";
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    value.time_since_epoch()).count();
  return std::to_string(ns);
}

inline std::optional<Phase5bAttemptDecision> phase5bAttemptDecisionForPhase(
  std::string_view scenario,
  DiagnosticPhase phase)
{
  if (scenario == "preselection" || scenario == "normal") {
    switch (phase) {
      case DiagnosticPhase::transaction_complete:
        return Phase5bAttemptDecision{true, "success"};
      case DiagnosticPhase::timeout_or_unresolved:
        return Phase5bAttemptDecision{false, "partial_unresolved"};
      case DiagnosticPhase::synchronization_complete:
        return Phase5bAttemptDecision{false, "partial_recovered_after_unresolved"};
      case DiagnosticPhase::before_fallback_close:
      case DiagnosticPhase::reconnect_complete:
        return Phase5bAttemptDecision{false, "failed_reconnect_after_unresolved"};
      default:
        return std::nullopt;
    }
  }

  if (scenario == "timeout" || scenario == "bounded-resync") {
    switch (phase) {
      case DiagnosticPhase::synchronization_complete:
        return Phase5bAttemptDecision{true, "success"};
      case DiagnosticPhase::before_fallback_close:
      case DiagnosticPhase::reconnect_complete:
        return Phase5bAttemptDecision{false, "failed_reconnect_after_unresolved"};
      default:
        return std::nullopt;
    }
  }

  if (scenario == "fallback-injected" && phase == DiagnosticPhase::reconnect_complete) {
    return Phase5bAttemptDecision{true, "success"};
  }

  return std::nullopt;
}

inline const char * diagnosticQueryKindName(DiagnosticQueryKind query)
{
  switch (query) {
    case DiagnosticQueryKind::firmware_id:
      return "FID";
    case DiagnosticQueryKind::fault_flags:
      return "FF";
    case DiagnosticQueryKind::motor_status_1:
      return "FM1";
    case DiagnosticQueryKind::motor_status_2:
      return "FM2";
    case DiagnosticQueryKind::status_flags:
      return "FS";
  }
  return "unknown";
}

inline const char * commandTransactionStatusName(CommandTransportStatus status)
{
  switch (status) {
    case CommandTransportStatus::success:
      return "success";
    case CommandTransportStatus::unresolved:
      return "unresolved";
    case CommandTransportStatus::failure:
      return "failure";
  }
  return "unknown";
}

inline const char * queryClassificationName(QueryLineClassification classification)
{
  switch (classification) {
    case QueryLineClassification::none:
      return "none";
    case QueryLineClassification::acknowledgement:
      return "acknowledgement";
    case QueryLineClassification::rejection:
      return "rejection";
    case QueryLineClassification::echo:
      return "echo";
    case QueryLineClassification::expected_reply:
      return "expected_reply";
    case QueryLineClassification::unexpected_reply:
      return "unexpected_reply";
  }
  return "unknown";
}

inline std::string concatenateCommandBytes(const std::vector<std::string> & commands)
{
  std::string combined;
  for (const auto & command : commands) {
    combined += command;
  }
  return combined;
}

inline std::string phase5bCommandTransactionJsonLine(
  const std::vector<std::string> & commands,
  const CommandTransactionResult & result)
{
  const auto transmitted = concatenateCommandBytes(commands);
  std::ostringstream line;
  line << "{\"type\":\"command_transaction\""
       << ",\"status\":\"" << commandTransactionStatusName(result.status) << "\""
       << ",\"expected_ack_count\":" << result.expected_acknowledgements
       << ",\"received_ack_count\":" << result.received_acknowledgements
       << ",\"write_fully_accepted\":"
       << (result.write_fully_accepted ? "true" : "false")
       << ",\"partial_line\":" << (result.partial_line ? "true" : "false")
       << ",\"quiet_verified\":"
       << (result.status == CommandTransportStatus::success ? "true" : "false")
       << ",\"monotonic_started_ns\":" << time_point_ns_or_null(result.started_at)
       << ",\"monotonic_write_accepted_ns\":"
       << time_point_ns_or_null(result.write_accepted_at)
       << ",\"monotonic_completed_ns\":" << time_point_ns_or_null(result.completed_at)
       << ",\"command_count\":" << commands.size()
       << ",\"transmitted_byte_count\":" << transmitted.size()
       << ",\"transmitted_raw\":\"" << jsonEscapeExactBytes(transmitted) << "\""
       << ",\"transmitted_hex\":\"" << bytesToHex(transmitted) << "\""
       << ",\"ack_byte_count\":" << result.raw_bytes.size()
       << ",\"ack_raw\":\"" << jsonEscapeExactBytes(result.raw_bytes) << "\""
       << ",\"ack_hex\":\"" << bytesToHex(result.raw_bytes) << "\""
       << ",\"reason\":\"" << jsonEscapeExactBytes(result.reason) << "\""
       << ",\"measurement_boundary\":\"os_library_write_acceptance_then_owned_ack_collection\""
       << "}";
  return line.str();
}

inline std::string phase5bStartupDrainJsonLine(const StartupDrainResult & result)
{
  std::ostringstream line;
  line << "{\"type\":\"startup_drain\""
       << ",\"synchronized\":" << (result.synchronized ? "true" : "false")
       << ",\"monotonic_started_ns\":" << time_point_ns_or_null(result.started_at)
       << ",\"monotonic_last_byte_ns\":" << time_point_ns_or_null(result.last_byte_at)
       << ",\"monotonic_completed_ns\":" << time_point_ns_or_null(result.completed_at)
       << ",\"delimiter_observed\":" << (result.delimiter_observed ? "true" : "false")
       << ",\"byte_count\":" << result.raw_bytes.size()
       << ",\"raw\":\"" << jsonEscapeExactBytes(result.raw_bytes) << "\""
       << ",\"hex\":\"" << bytesToHex(result.raw_bytes) << "\""
       << ",\"reason\":\"" << jsonEscapeExactBytes(result.reason) << "\""
       << "}";
  return line.str();
}

inline std::string phase5bQueryTraceJsonLine(const QueryTraceEvent & event)
{
  std::ostringstream line;
  line << "{\"type\":\"query_trace\""
       << ",\"command\":\"" << jsonEscapeExactBytes(event.command) << "\""
       << ",\"command_hex\":\"" << bytesToHex(event.command) << "\""
       << ",\"expected_prefix\":\"" << jsonEscapeExactBytes(event.expected_prefix) << "\""
       << ",\"success\":" << (event.success ? "true" : "false")
       << ",\"classification\":\"" << queryClassificationName(event.classification) << "\""
       << ",\"monotonic_started_ns\":" << time_point_ns_or_null(event.started_at)
       << ",\"monotonic_write_started_ns\":" << time_point_ns_or_null(event.write_started_at)
       << ",\"monotonic_write_accepted_ns\":" << time_point_ns_or_null(event.write_accepted_at)
       << ",\"monotonic_first_byte_ns\":" << time_point_ns_or_null(event.first_byte_at)
       << ",\"monotonic_last_byte_ns\":" << time_point_ns_or_null(event.last_byte_at)
       << ",\"monotonic_completed_ns\":" << time_point_ns_or_null(event.completed_at)
       << ",\"delimiter_observed\":" << (event.delimiter_observed ? "true" : "false")
       << ",\"byte_count\":" << event.raw_bytes.size()
       << ",\"raw\":\"" << jsonEscapeExactBytes(event.raw_bytes) << "\""
       << ",\"hex\":\"" << bytesToHex(event.raw_bytes) << "\""
       << ",\"response\":\"" << jsonEscapeExactBytes(event.response) << "\""
       << ",\"reason\":\"" << jsonEscapeExactBytes(event.reason) << "\""
       << "}";
  return line.str();
}

inline std::string phase5bDiagnosticResultJsonLine(const DiagnosticResultEvent & event)
{
  const auto status = event.status == DiagnosticTransportStatus::success ? "success" :
    (event.status == DiagnosticTransportStatus::timeout ? "timeout" : "failure");
  const auto framing_state = event.framing_state == SerialFramingState::synchronized ?
    "synchronized" : "unresolved";
  std::ostringstream line;
  line << "{\"type\":\"diagnostic_result\""
       << ",\"query\":\"" << diagnosticQueryKindName(event.query) << "\""
       << ",\"status\":\"" << status << "\""
       << ",\"framing_state\":\"" << framing_state << "\""
       << ",\"correlation\":" << event.correlation_id
       << ",\"generation\":" << event.connection_generation
       << ",\"monotonic_started_ns\":" << time_point_ns_or_null(event.started_at)
       << ",\"monotonic_write_accepted_ns\":"
       << time_point_ns_or_null(event.write_accepted_at)
       << ",\"monotonic_first_byte_ns\":" << time_point_ns_or_null(event.first_byte_at)
       << ",\"monotonic_last_byte_ns\":" << time_point_ns_or_null(event.last_byte_at)
       << ",\"monotonic_timeout_ns\":" << time_point_ns_or_null(event.timeout_at)
       << ",\"monotonic_completed_ns\":" << time_point_ns_or_null(event.completed_at)
       << ",\"delimiter_observed\":" << (event.delimiter_observed ? "true" : "false")
       << ",\"byte_count\":" << event.raw_bytes.size()
       << ",\"raw\":\"" << jsonEscapeExactBytes(event.raw_bytes) << "\""
       << ",\"hex\":\"" << bytesToHex(event.raw_bytes) << "\""
       << ",\"response\":\"" << jsonEscapeExactBytes(event.response) << "\""
       << ",\"reason\":\"" << jsonEscapeExactBytes(event.reason) << "\""
       << "}";
  return line.str();
}

inline std::string phase5bAttemptResultJsonLine(
  int attempt,
  std::string_view scenario,
  std::string_view outcome,
  bool success,
  std::string_view connection_state,
  std::string_view framing_state,
  const std::optional<Phase5bDiagnosticEvidence> & diagnostic_evidence)
{
  std::ostringstream line;
  line << "{\"type\":\"attempt_result\",\"attempt\":" << attempt <<
    ",\"scenario\":\"" << scenario <<
    "\",\"outcome\":\"" << outcome <<
    "\",\"success\":" << (success ? "true" : "false") <<
    ",\"connection_state\":\"" << connection_state <<
    "\",\"framing_state\":\"" << framing_state << "\"";
  if (diagnostic_evidence.has_value()) {
    line << ",\"diagnostic_correlation_id\":" <<
      diagnostic_evidence->diagnostic_correlation_id <<
      ",\"diagnostic_connection_generation\":" <<
      diagnostic_evidence->diagnostic_connection_generation <<
      ",\"diagnostic_started_ns\":" << diagnostic_evidence->diagnostic_started_ns <<
      ",\"diagnostic_write_accepted_ns\":" <<
      diagnostic_evidence->diagnostic_write_accepted_ns <<
      ",\"diagnostic_first_byte_ns\":" << diagnostic_evidence->diagnostic_first_byte_ns <<
      ",\"diagnostic_last_byte_ns\":" << diagnostic_evidence->diagnostic_last_byte_ns <<
      ",\"diagnostic_timeout_ns\":" << diagnostic_evidence->diagnostic_timeout_ns <<
      ",\"diagnostic_delimiter_observed\":" <<
      (diagnostic_evidence->diagnostic_delimiter_observed ? "true" : "false") <<
      ",\"diagnostic_byte_count\":" << diagnostic_evidence->diagnostic_byte_count <<
      ",\"diagnostic_raw\":\"" << jsonEscapeExactBytes(diagnostic_evidence->diagnostic_raw) <<
      "\",\"diagnostic_raw_hex\":\"" <<
      diagnostic_evidence->diagnostic_raw_hex <<
      "\",\"diagnostic_failure_reason\":\"" <<
      jsonEscapeExactBytes(diagnostic_evidence->diagnostic_failure_reason) << "\"";
  }
  line << "}";
  return line.str();
}

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__PHASE5B_HARNESS_LOGIC_HPP_
