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

#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <charconv>
#include <exception>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace roboteq_ros2_driver
{
namespace
{

constexpr std::size_t kMaxDiagnosticResponseBytes = 256;

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool is_allowed_diagnostic_command(const std::string & command)
{
  return command == "?FID\r" || command == "?FF\r" || command == "?FM 1\r" ||
         command == "?FM 2\r" || command == "?FS\r";
}

std::string expected_prefix_for_diagnostic(const std::string & command)
{
  if (command == "?FID\r") {
    return "FID=";
  }
  if (command == "?FF\r") {
    return "FF=";
  }
  if (command == "?FM 1\r" || command == "?FM 2\r") {
    return "FM=";
  }
  if (command == "?FS\r") {
    return "FS=";
  }
  return "";
}

bool valid_diagnostic_response(
  const std::string & command, const std::string & expected_prefix, const std::string & response)
{
  if (!starts_with(response, expected_prefix) || response.size() == expected_prefix.size()) {
    return false;
  }
  const auto payload = response.substr(expected_prefix.size());
  if (command == "?FID\r") {
    return std::all_of(
      payload.begin(), payload.end(), [](unsigned char ch) {return ch >= 0x20 && ch <= 0x7e;});
  }
  int value = 0;
  const auto parsed = std::from_chars(payload.data(), payload.data() + payload.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == payload.data() + payload.size();
}

bool one_complete_line(const std::string & raw, std::string & line)
{
  line.clear();
  if (raw.empty() || (raw.back() != '\r' && raw.back() != '\n')) {
    return false;
  }
  if (raw.size() == 1 ||
    raw.find_first_of("\r\n") != raw.size() - 1)
  {
    return false;
  }
  line.assign(raw.data(), raw.size() - 1);
  return true;
}

}  // namespace

std::string strip_roboteq_line_endings(const std::string & text)
{
  std::string stripped = text;
  while (!stripped.empty() && (stripped.back() == '\r' || stripped.back() == '\n')) {
    stripped.pop_back();
  }
  return stripped;
}

RoboteqSerialTransport::RoboteqSerialTransport(SerialTransportConfig config)
: config_(std::move(config))
{
  serial_.setPort(config_.port);
  serial_.setBaudrate(static_cast<uint32_t>(config_.baud));
  const auto timeout_ms = static_cast<uint32_t>(
    std::max(config_.read_timeout, config_.write_timeout).count());
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
  serial_.setTimeout(timeout);
}

bool RoboteqSerialTransport::open(std::string & error)
{
  try {
    if (!serial_.isOpen()) {
      serial_.open();
    }
    if (!serial_.isOpen()) {
      error = "serial port did not open";
      return false;
    }
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

void RoboteqSerialTransport::close() noexcept
{
  try {
    if (serial_.isOpen()) {
      serial_.close();
    }
  } catch (...) {
  }
}

bool RoboteqSerialTransport::isOpen() const noexcept
{
  try {
    return serial_.isOpen();
  } catch (...) {
    return false;
  }
}

bool RoboteqSerialTransport::sendCommands(
  const std::vector<std::string> & commands, std::string & error)
{
  if (!isOpen()) {
    error = "serial port is not open";
    return false;
  }

  try {
    for (const auto & command : commands) {
      const std::size_t written = serial_.write(command);
      if (written != command.size()) {
        std::ostringstream stream;
        stream << "partial serial write: " << written << " of " << command.size() << " bytes";
        error = stream.str();
        return false;
      }
    }
    serial_.flush();
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

bool RoboteqSerialTransport::query(
  const std::string & command,
  const std::string & expected_prefix,
  std::string & response,
  std::string & error)
{
  if (!isOpen()) {
    error = "serial port is not open";
    return false;
  }

  try {
    const std::size_t written = serial_.write(command);
    if (written != command.size()) {
      std::ostringstream stream;
      stream << "partial serial query write: " << written << " of " << command.size() << " bytes";
      error = stream.str();
      return false;
    }
    serial_.flush();
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + config_.transaction_timeout;
  const std::string stripped_command = strip_roboteq_line_endings(command);
  std::size_t total_bytes = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    std::string line;
    if (!readLine(deadline, line, error)) {
      if (!response.empty()) {
        const std::string partial = line.empty() ? "" : "; subsequent_partial=" + line;
        error = "unexpected response prefix: expected " + expected_prefix + " received " +
          response + partial + "; subsequent_error=" + error;
      } else if (!line.empty()) {
        response = line;
      }
      return false;
    }
    if (line.empty()) {
      continue;
    }

    total_bytes += line.size();
    if (total_bytes > config_.max_response_bytes) {
      error = "serial query response exceeded maximum size";
      return false;
    }

    if (line == stripped_command || line == "+") {
      continue;
    }
    if (line == "-") {
      response = line;
      error = "Roboteq rejected query";
      return false;
    }
    if (starts_with(line, expected_prefix)) {
      response = line;
      return true;
    }
    response = line;
  }

  if (response.empty()) {
    error = "serial query timed out waiting for " + expected_prefix;
  } else {
    error = "unexpected response prefix: expected " + expected_prefix + " received " + response;
  }
  return false;
}

bool RoboteqSerialTransport::writeReadOnlyDiagnostic(
  const std::string & command, std::string & error)
{
  if (!is_allowed_diagnostic_command(command)) {
    error = "diagnostic query is not in the read-only allowlist";
    return false;
  }
  if (!isOpen()) {
    error = "serial port is not open";
    return false;
  }
  try {
    const auto written = serial_.write(command);
    if (written != command.size()) {
      std::ostringstream stream;
      stream << "partial serial diagnostic write: " << written << " of " << command.size() <<
        " bytes";
      error = stream.str();
      return false;
    }
    serial_.flush();
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

DiagnosticTransactionResult RoboteqSerialTransport::diagnosticQuery(
  const DiagnosticTransaction & transaction)
{
  DiagnosticTransactionResult result;
  result.started_at = std::chrono::steady_clock::now();
  if (transaction.expected_prefix != expected_prefix_for_diagnostic(transaction.command)) {
    result.reason = "diagnostic query expected prefix does not match the read-only allowlist";
    return result;
  }
  if (!writeReadOnlyDiagnostic(transaction.command, result.reason)) {
    return result;
  }

  const auto deadline = result.started_at + transaction.timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (serial_.available() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      char ch = 0;
      if (serial_.read(reinterpret_cast<uint8_t *>(&ch), 1) == 0) {
        continue;
      }
      result.raw_bytes.push_back(ch);
      if (result.raw_bytes.size() >
        std::min(config_.max_response_bytes, kMaxDiagnosticResponseBytes))
      {
        result.reason = "diagnostic response exceeded maximum size";
        return result;
      }
      if (ch == '\r' || ch == '\n') {
        std::string line;
        if (!one_complete_line(result.raw_bytes, line)) {
          result.reason = "diagnostic response framing is ambiguous";
          return result;
        }
        result.response = line;
        if (!valid_diagnostic_response(
            transaction.command, transaction.expected_prefix, result.response))
        {
          result.reason = "diagnostic response prefix or payload is malformed";
          return result;
        }
        while (std::chrono::steady_clock::now() < deadline) {
          try {
            if (serial_.available() == 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
              continue;
            }
            char extra = 0;
            if (serial_.read(reinterpret_cast<uint8_t *>(&extra), 1) != 0) {
              result.raw_bytes.push_back(extra);
              result.reason = "extra bytes after diagnostic response";
              return result;
            }
          } catch (const std::exception & ex) {
            result.reason = ex.what();
            return result;
          }
        }
        result.status = DiagnosticTransportStatus::success;
        return result;
      }
    } catch (const std::exception & ex) {
      result.reason = ex.what();
      return result;
    }
  }
  result.status = DiagnosticTransportStatus::timeout;
  result.reason = result.raw_bytes.empty() ? "diagnostic query timed out" :
    "diagnostic query timed out with a partial response";
  return result;
}

bool RoboteqSerialTransport::readRawUntilQuiet(
  const std::chrono::steady_clock::time_point & not_before,
  const std::chrono::steady_clock::time_point & absolute_deadline,
  std::chrono::milliseconds quiet_period,
  std::size_t max_bytes,
  std::string & raw,
  std::string & error)
{
  raw.clear();
  while (std::chrono::steady_clock::now() < not_before) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto quiet_deadline = not_before + quiet_period;
  while (std::chrono::steady_clock::now() <= absolute_deadline) {
    if (std::chrono::steady_clock::now() >= quiet_deadline) {
      return true;
    }
    try {
      if (serial_.available() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      char ch = 0;
      if (serial_.read(reinterpret_cast<uint8_t *>(&ch), 1) == 0) {
        continue;
      }
      raw.push_back(ch);
      if (raw.size() > max_bytes) {
        error = "bounded diagnostic drain exceeded maximum size";
        return false;
      }
      quiet_deadline = std::chrono::steady_clock::now() + quiet_period;
    } catch (const std::exception & ex) {
      error = ex.what();
      return false;
    }
  }
  error = "bounded diagnostic drain reached its absolute deadline";
  return false;
}

DiagnosticRecoveryResult RoboteqSerialTransport::boundedDiagnosticRecovery(
  const DiagnosticTransaction & timed_out_transaction,
  std::chrono::steady_clock::time_point timed_out_query_started_at,
  const DiagnosticTransaction & synchronization_transaction,
  const DiagnosticRecoveryBounds & bounds)
{
  DiagnosticRecoveryResult result;
  if (!is_allowed_diagnostic_command(timed_out_transaction.command) ||
    !is_allowed_diagnostic_command(synchronization_transaction.command) ||
    timed_out_transaction.expected_prefix == synchronization_transaction.expected_prefix)
  {
    result.reason = "invalid or non-distinguishable diagnostic recovery query";
    return result;
  }

  std::string drain_error;
  if (!readRawUntilQuiet(
      timed_out_query_started_at + bounds.delayed_reply_horizon,
      timed_out_query_started_at + bounds.drain_absolute_limit,
      bounds.drain_quiet_period,
      bounds.max_drain_bytes,
      result.drained_raw_bytes,
      drain_error))
  {
    result.reason = drain_error;
    return result;
  }
  if (!result.drained_raw_bytes.empty()) {
    std::string delayed_line;
    if (!one_complete_line(result.drained_raw_bytes, delayed_line) ||
      !valid_diagnostic_response(
        timed_out_transaction.command, timed_out_transaction.expected_prefix, delayed_line))
    {
      result.reason = "drained delayed response is partial, malformed, or ambiguous";
      return result;
    }
  }

  DiagnosticTransaction sync = synchronization_transaction;
  sync.timeout = bounds.synchronization_timeout;
  const auto sync_result = diagnosticQuery(sync);
  result.synchronization_response = sync_result.response;
  if (sync_result.status != DiagnosticTransportStatus::success) {
    result.drained_raw_bytes += sync_result.raw_bytes;
    result.reason = "synchronization query failed: " + sync_result.reason;
    return result;
  }

  std::string extra_raw;
  if (!readRawUntilQuiet(
      std::chrono::steady_clock::now(),
      std::chrono::steady_clock::now() + bounds.post_sync_absolute_limit,
      bounds.post_sync_quiet_period,
      bounds.max_response_bytes,
      extra_raw,
      drain_error))
  {
    result.drained_raw_bytes += extra_raw;
    result.reason = "post-synchronization framing check failed: " + drain_error;
    return result;
  }
  if (!extra_raw.empty()) {
    result.drained_raw_bytes += extra_raw;
    result.reason = "extra bytes after synchronization response";
    return result;
  }
  result.synchronized = true;
  return result;
}

bool RoboteqSerialTransport::readLine(
  const std::chrono::steady_clock::time_point & deadline,
  std::string & line,
  std::string & error)
{
  line.clear();
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (serial_.available() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      char ch = 0;
      if (serial_.read(reinterpret_cast<uint8_t *>(&ch), 1) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (ch == '\r' || ch == '\n') {
        if (!line.empty()) {
          return true;
        }
        continue;
      }
      line.push_back(ch);
      if (line.size() > config_.max_response_bytes) {
        error = "serial query line exceeded maximum size";
        return false;
      }
    } catch (const std::exception & ex) {
      error = ex.what();
      return false;
    }
  }

  error = "serial query timed out before line delimiter";
  return false;
}

}  // namespace roboteq_ros2_driver
