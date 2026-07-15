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

#ifndef ROBOTEQ_ROS2_DRIVER__PHASE5B_ACK_CHARACTERIZATION_LOGIC_HPP_
#define ROBOTEQ_ROS2_DRIVER__PHASE5B_ACK_CHARACTERIZATION_LOGIC_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace roboteq_ros2_driver
{

enum class AckCharacterizationMode
{
  h1_single,
  h2_batch,
  h3_stop_then_fid,
  h3_stop_then_ff,
};

enum class ZeroCommandKind
{
  g1_zero,
  g2_zero,
  s1_zero,
  s2_zero,
};

struct AckCharacterizationArguments
{
  std::string port;
  int baud{0};
  std::string output;
  AckCharacterizationMode mode{AckCharacterizationMode::h1_single};
  std::optional<ZeroCommandKind> command;
  int attempts{0};
  int capture_deadline_ms{0};
  int quiet_period_ms{0};
  int max_bytes{0};
  int query_deadline_ms{0};
};

struct ByteObservation
{
  char value{0};
  std::chrono::steady_clock::time_point timestamp{};
  bool after_query_write{false};
};

struct CapturedLine
{
  std::string raw_bytes;
  std::chrono::steady_clock::time_point completed_at{};
  bool after_query_write{false};
};

struct CaptureAnalysis
{
  std::vector<CapturedLine> lines;
  std::vector<int64_t> inter_line_gap_ns;
  std::string trailing_partial;
  bool delimiter_observed{false};
  std::size_t ack_count{0};
  std::size_t ack_count_after_query_write{0};
  std::size_t expected_query_reply_count{0};
  std::size_t unexpected_line_count{0};
};

inline std::string ackCharacterizationModeName(AckCharacterizationMode mode)
{
  switch (mode) {
    case AckCharacterizationMode::h1_single: return "h1-single";
    case AckCharacterizationMode::h2_batch: return "h2-batch";
    case AckCharacterizationMode::h3_stop_then_fid: return "h3-stop-then-fid";
    case AckCharacterizationMode::h3_stop_then_ff: return "h3-stop-then-ff";
  }
  return "unknown";
}

inline std::string zeroCommandSelector(ZeroCommandKind command)
{
  switch (command) {
    case ZeroCommandKind::g1_zero: return "G1";
    case ZeroCommandKind::g2_zero: return "G2";
    case ZeroCommandKind::s1_zero: return "S1";
    case ZeroCommandKind::s2_zero: return "S2";
  }
  return "unknown";
}

inline std::string zeroCommandBytes(ZeroCommandKind command)
{
  switch (command) {
    case ZeroCommandKind::g1_zero: return "!G 1 0\r";
    case ZeroCommandKind::g2_zero: return "!G 2 0\r";
    case ZeroCommandKind::s1_zero: return "!S 1 0\r";
    case ZeroCommandKind::s2_zero: return "!S 2 0\r";
  }
  return "";
}

inline std::string stopBatchBytes()
{
  return "!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r";
}

inline std::string queryBytesForMode(AckCharacterizationMode mode)
{
  switch (mode) {
    case AckCharacterizationMode::h3_stop_then_fid: return "?FID\r";
    case AckCharacterizationMode::h3_stop_then_ff: return "?FF\r";
    default: return "";
  }
}

inline std::string queryExpectedPrefixForMode(AckCharacterizationMode mode)
{
  switch (mode) {
    case AckCharacterizationMode::h3_stop_then_fid: return "FID=";
    case AckCharacterizationMode::h3_stop_then_ff: return "FF=";
    default: return "";
  }
}

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

inline std::string visibleBytes(std::string_view bytes)
{
  std::string visible;
  visible.reserve(bytes.size() * 4);
  for (const unsigned char ch : bytes) {
    switch (ch) {
      case '\r':
        visible += "\\r";
        break;
      case '\n':
        visible += "\\n";
        break;
      case '\t':
        visible += "\\t";
        break;
      case '\\':
        visible += "\\\\";
        break;
      default:
        if (ch >= 0x20 && ch <= 0x7e) {
          visible.push_back(static_cast<char>(ch));
        } else {
          std::ostringstream stream;
          stream << "\\x";
          static constexpr char kHexDigits[] = "0123456789ABCDEF";
          stream << kHexDigits[ch >> 4] << kHexDigits[ch & 0x0f];
          visible += stream.str();
        }
        break;
    }
  }
  return visible;
}

inline std::string stripLineEndings(const std::string & text)
{
  std::string stripped = text;
  while (!stripped.empty() && (stripped.back() == '\r' || stripped.back() == '\n')) {
    stripped.pop_back();
  }
  return stripped;
}

inline CaptureAnalysis analyzeCapture(
  const std::vector<ByteObservation> & observations,
  const std::optional<std::string> & expected_query_prefix = std::nullopt)
{
  CaptureAnalysis analysis;
  std::string current_line;
  bool current_line_after_query_write = false;
  std::optional<std::chrono::steady_clock::time_point> previous_line_completed_at;
  for (const auto & observation : observations) {
    current_line.push_back(observation.value);
    current_line_after_query_write = current_line_after_query_write ||
      observation.after_query_write;
    if (observation.value == '\r' || observation.value == '\n') {
      analysis.delimiter_observed = true;
      const auto stripped = stripLineEndings(current_line);
      if (!stripped.empty()) {
        analysis.lines.push_back(
          CapturedLine{current_line, observation.timestamp, current_line_after_query_write});
        if (previous_line_completed_at.has_value()) {
          analysis.inter_line_gap_ns.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              observation.timestamp - *previous_line_completed_at).count());
        }
        previous_line_completed_at = observation.timestamp;
        if (stripped == "+") {
          ++analysis.ack_count;
          if (current_line_after_query_write) {
            ++analysis.ack_count_after_query_write;
          }
        } else if (
          expected_query_prefix.has_value() &&
          stripped.rfind(*expected_query_prefix, 0) == 0)
        {
          ++analysis.expected_query_reply_count;
        } else {
          ++analysis.unexpected_line_count;
        }
      }
      current_line.clear();
      current_line_after_query_write = false;
    }
  }
  analysis.trailing_partial = current_line;
  return analysis;
}

inline std::string timePointNsOrNull(const std::chrono::steady_clock::time_point & value)
{
  if (value == std::chrono::steady_clock::time_point{}) {
    return "null";
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    value.time_since_epoch()).count();
  return std::to_string(ns);
}

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__PHASE5B_ACK_CHARACTERIZATION_LOGIC_HPP_
