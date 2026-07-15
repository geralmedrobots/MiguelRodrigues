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

#include <charconv>

#include <algorithm>
#include <chrono>
#include <cstdint>
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

enum class RoboteqLineKind
{
  acknowledgement,
  rejection,
  echo,
  expected_reply,
  unexpected_reply,
};

RoboteqLineKind classify_line(
  const std::string & line, const std::string & command, const std::string & expected_prefix)
{
  if (line == "+") {
    return RoboteqLineKind::acknowledgement;
  }
  if (line == "-") {
    return RoboteqLineKind::rejection;
  }
  if (!command.empty() && line == strip_roboteq_line_endings(command)) {
    return RoboteqLineKind::echo;
  }
  if (!expected_prefix.empty() && starts_with(line, expected_prefix)) {
    return RoboteqLineKind::expected_reply;
  }
  return RoboteqLineKind::unexpected_reply;
}

QueryLineClassification query_classification(RoboteqLineKind kind)
{
  switch (kind) {
    case RoboteqLineKind::acknowledgement:
      return QueryLineClassification::acknowledgement;
    case RoboteqLineKind::rejection:
      return QueryLineClassification::rejection;
    case RoboteqLineKind::echo:
      return QueryLineClassification::echo;
    case RoboteqLineKind::expected_reply:
      return QueryLineClassification::expected_reply;
    case RoboteqLineKind::unexpected_reply:
      return QueryLineClassification::unexpected_reply;
  }
  return QueryLineClassification::none;
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

bool startup_input_is_well_formed(const std::string & raw, std::string & reason)
{
  bool seen_printable = false;
  for (const unsigned char ch : raw) {
    if (ch == '\r' || ch == '\n') {
      continue;
    }
    if (!seen_printable && ch == '\0') {
      continue;
    }
    if (ch < 0x20 || ch > 0x7e) {
      reason = "startup input contains malformed non-printable bytes";
      return false;
    }
    seen_printable = true;
  }
  return true;
}

void observe_diagnostic(
  const DiagnosticTransaction & transaction,
  DiagnosticPhase phase,
  std::size_t byte_count = 0) noexcept
{
  if (!transaction.observer) {
    return;
  }
  try {
    transaction.observer(
      DiagnosticPhaseEvent{
          phase, std::chrono::steady_clock::now(), transaction.correlation_id,
          transaction.connection_generation, transaction.command, byte_count});
  } catch (...) {
    // Validation observability must never alter transport behavior.
  }
}

void observe_query(
  const std::function<void(const QueryTraceEvent &)> & observer,
  const QueryTraceEvent & event) noexcept
{
  if (!observer) {
    return;
  }
  try {
    observer(event);
  } catch (...) {
    // Validation observability must never alter transport behavior.
  }
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

StartupDrainResult RoboteqSerialTransport::drainStartupInput(const StartupDrainBounds & bounds)
{
  StartupDrainResult result;
  result.started_at = std::chrono::steady_clock::now();
  const auto finish = [&result](bool synchronized, const std::string & reason) {
      result.synchronized = synchronized;
      result.reason = reason;
      result.completed_at = std::chrono::steady_clock::now();
      return result;
    };

  if (!isOpen()) {
    return finish(false, "serial port is not open");
  }
  if (bounds.absolute_limit <= std::chrono::milliseconds::zero() ||
    bounds.quiet_period < std::chrono::milliseconds::zero() ||
    bounds.max_bytes == 0)
  {
    return finish(false, "invalid startup drain bounds");
  }

  std::string error;
  const auto absolute_deadline = result.started_at + bounds.absolute_limit;
  if (!readRawUntilQuiet(
      result.started_at, absolute_deadline, bounds.quiet_period, bounds.max_bytes,
      result.raw_bytes, result.last_byte_at, result.delimiter_observed, error))
  {
    return finish(false, error);
  }
  if (!result.raw_bytes.empty() &&
    result.raw_bytes.back() != '\r' && result.raw_bytes.back() != '\n')
  {
    return finish(false, "startup input ended with a partial line");
  }
  std::string malformed_reason;
  if (!startup_input_is_well_formed(result.raw_bytes, malformed_reason)) {
    return finish(false, malformed_reason);
  }
  return finish(true, "");
}

CommandTransactionResult RoboteqSerialTransport::commandTransaction(
  const std::vector<std::string> & commands,
  const CommandTransactionBounds & bounds)
{
  CommandTransactionResult result;
  result.started_at = std::chrono::steady_clock::now();
  result.expected_acknowledgements = commands.size();
  const auto finish = [&result](CommandTransportStatus status, const std::string & reason) {
      result.status = status;
      result.reason = reason;
      result.completed_at = std::chrono::steady_clock::now();
      return result;
    };

  if (commands.empty()) {
    return finish(CommandTransportStatus::failure, "empty command transaction");
  }
  if (!isOpen()) {
    return finish(CommandTransportStatus::failure, "serial port is not open");
  }
  if (bounds.acknowledgement_deadline <= std::chrono::milliseconds::zero() ||
    bounds.post_ack_quiet_period < std::chrono::milliseconds::zero() ||
    bounds.max_response_bytes == 0)
  {
    return finish(CommandTransportStatus::failure, "invalid command transaction bounds");
  }

  bool write_started = false;
  try {
    // The complete batch is accepted by the library before acknowledgement ownership begins.
    for (const auto & command : commands) {
      write_started = true;
      const auto written = serial_.write(command);
      if (written != command.size()) {
        std::ostringstream stream;
        stream << "partial serial write: " << written << " of " << command.size() << " bytes";
        return finish(CommandTransportStatus::unresolved, stream.str());
      }
    }
    result.write_accepted_at = std::chrono::steady_clock::now();
    result.write_fully_accepted = true;
  } catch (const std::exception & ex) {
    return finish(
      write_started ? CommandTransportStatus::unresolved : CommandTransportStatus::failure,
      ex.what());
  }

  const auto deadline = result.write_accepted_at + bounds.acknowledgement_deadline;
  auto quiet_deadline = std::chrono::steady_clock::time_point::max();
  std::string line;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (serial_.available() == 0) {
        if (result.received_acknowledgements == result.expected_acknowledgements &&
          std::chrono::steady_clock::now() >= quiet_deadline)
        {
          return finish(CommandTransportStatus::success, "");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      char ch = 0;
      if (serial_.read(reinterpret_cast<uint8_t *>(&ch), 1) == 0) {
        continue;
      }
      result.raw_bytes.push_back(ch);
      if (result.raw_bytes.size() > bounds.max_response_bytes) {
        return finish(CommandTransportStatus::unresolved, "command replies exceeded byte cap");
      }
      if (ch != '\r' && ch != '\n') {
        line.push_back(ch);
        continue;
      }
      if (classify_line(line, "", "") != RoboteqLineKind::acknowledgement) {
        return finish(
          CommandTransportStatus::unresolved,
          "unexpected or malformed line while owning command acknowledgements");
      }
      line.clear();
      ++result.received_acknowledgements;
      if (result.received_acknowledgements > result.expected_acknowledgements) {
        return finish(CommandTransportStatus::unresolved, "extra command acknowledgement");
      }
      if (result.received_acknowledgements == result.expected_acknowledgements) {
        quiet_deadline = std::chrono::steady_clock::now() + bounds.post_ack_quiet_period;
        if (quiet_deadline > deadline) {
          return finish(
            CommandTransportStatus::unresolved,
            "post-acknowledgement quiet period exceeds absolute deadline");
        }
      }
    } catch (const std::exception & ex) {
      return finish(CommandTransportStatus::failure, ex.what());
    }
  }
  result.partial_line = !line.empty();
  if (result.partial_line) {
    return finish(
      CommandTransportStatus::unresolved,
      "partial command acknowledgement at deadline");
  }
  if (result.received_acknowledgements != result.expected_acknowledgements) {
    return finish(CommandTransportStatus::unresolved, "command acknowledgement deadline expired");
  }
  return finish(CommandTransportStatus::unresolved, "command acknowledgement quiet check expired");
}

bool RoboteqSerialTransport::query(
  const std::string & command,
  const std::string & expected_prefix,
  std::string & response,
  std::string & error)
{
  QueryTraceEvent trace;
  trace.command = command;
  trace.expected_prefix = expected_prefix;
  trace.started_at = std::chrono::steady_clock::now();
  const auto finish = [&](bool success) {
      trace.success = success;
      trace.response = response;
      trace.reason = error;
      trace.completed_at = std::chrono::steady_clock::now();
      observe_query(config_.query_observer, trace);
      return success;
    };

  if (!isOpen()) {
    error = "serial port is not open";
    return finish(false);
  }

  try {
    trace.write_started_at = std::chrono::steady_clock::now();
    const std::size_t written = serial_.write(command);
    if (written != command.size()) {
      std::ostringstream stream;
      stream << "partial serial query write: " << written << " of " << command.size() << " bytes";
      error = stream.str();
      return finish(false);
    }
    trace.write_accepted_at = std::chrono::steady_clock::now();
  } catch (const std::exception & ex) {
    error = ex.what();
    return finish(false);
  }

  const auto deadline = std::chrono::steady_clock::now() + config_.transaction_timeout;
  const std::string stripped_command = strip_roboteq_line_endings(command);
  std::size_t total_bytes = 0;
  std::string line;

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
      trace.raw_bytes.push_back(ch);
      trace.last_byte_at = std::chrono::steady_clock::now();
      if (trace.first_byte_at == std::chrono::steady_clock::time_point{}) {
        trace.first_byte_at = trace.last_byte_at;
      }
      if (trace.raw_bytes.size() > config_.max_response_bytes) {
        error = "serial query response exceeded maximum size";
        return finish(false);
      }
      if (ch != '\r' && ch != '\n') {
        line.push_back(ch);
        continue;
      }
      trace.delimiter_observed = true;
    } catch (const std::exception & ex) {
      error = ex.what();
      return finish(false);
    }

    if (line.empty()) {
      continue;
    }

    total_bytes += line.size();
    if (total_bytes > config_.max_response_bytes) {
      error = "serial query response exceeded maximum size";
      return finish(false);
    }

    const auto line_kind = classify_line(line, stripped_command, expected_prefix);
    trace.classification = query_classification(line_kind);
    switch (line_kind) {
      case RoboteqLineKind::echo:
        line.clear();
        continue;
      case RoboteqLineKind::acknowledgement:
        response = line;
        error = "unowned command acknowledgement encountered during query";
        return finish(false);
      case RoboteqLineKind::rejection:
        response = line;
        error = "Roboteq rejected query";
        return finish(false);
      case RoboteqLineKind::expected_reply:
        {
          response = line;
          std::string extra;
          std::chrono::steady_clock::time_point last_byte;
          bool delimiter = false;
          const auto quiet_started = std::chrono::steady_clock::now();
          const auto quiet_deadline = quiet_started + config_.post_reply_quiet_period;
          const bool quiet_complete = quiet_deadline > quiet_started && readRawUntilQuiet(
            quiet_started, quiet_deadline, config_.post_reply_quiet_period,
            config_.max_response_bytes, extra, last_byte, delimiter, error);
          trace.raw_bytes += extra;
          if (last_byte != std::chrono::steady_clock::time_point{}) {
            trace.last_byte_at = last_byte;
          }
          trace.delimiter_observed = trace.delimiter_observed || delimiter;
          if (!quiet_complete) {
            if (!extra.empty()) {
              response += "; extra_raw=" + extra;
              error = "extra bytes after query response; quiet verification failed: " + error;
              return finish(false);
            }
            if (error.empty()) {
              error = "query response left no bounded quiet-verification interval";
            }
            return finish(false);
          }
          if (!extra.empty()) {
            error = "extra bytes after query response";
            response += "; extra_raw=" + extra;
            return finish(false);
          }
          return finish(true);
        }
      case RoboteqLineKind::unexpected_reply:
        response = line;
        error = "unexpected response line during query: " + line;
        return finish(false);
    }
  }

  if (response.empty()) {
    if (!line.empty()) {
      response = line;
      error = "serial query timed out before line delimiter";
      return finish(false);
    }
    error = "serial query timed out waiting for " + expected_prefix;
  } else {
    error = "unexpected response prefix: expected " + expected_prefix + " received " + response;
  }
  return finish(false);
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
    // Diagnostic events are emitted by diagnosticQuery, which owns the correlation context.
    const auto written = serial_.write(command);
    if (written != command.size()) {
      std::ostringstream stream;
      stream << "partial serial diagnostic write: " << written << " of " << command.size() <<
        " bytes";
      error = stream.str();
      return false;
    }
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
  bool delimiter_observed = false;
  const auto complete_unresolved = [&]() {
      result.completed_at = std::chrono::steady_clock::now();
      result.timeout_at = result.completed_at;
      result.delimiter_observed = delimiter_observed;
      observe_diagnostic(
        transaction, DiagnosticPhase::timeout_or_unresolved, result.raw_bytes.size());
      return result;
    };
  if (transaction.expected_prefix != expected_prefix_for_diagnostic(transaction.command)) {
    result.reason = "diagnostic query expected prefix does not match the read-only allowlist";
    return complete_unresolved();
  }
  observe_diagnostic(transaction, DiagnosticPhase::write_started);
  if (!writeReadOnlyDiagnostic(transaction.command, result.reason)) {
    return complete_unresolved();
  }
  result.write_accepted_at = std::chrono::steady_clock::now();
  observe_diagnostic(transaction, DiagnosticPhase::write_accepted, transaction.command.size());

  const auto deadline = result.started_at + transaction.timeout;
  observe_diagnostic(transaction, DiagnosticPhase::waiting_for_first_byte);
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
      result.last_byte_at = std::chrono::steady_clock::now();
      if (result.first_byte_at == std::chrono::steady_clock::time_point{}) {
        result.first_byte_at = std::chrono::steady_clock::now();
        observe_diagnostic(transaction, DiagnosticPhase::first_byte_received, 1);
      }
      if (result.raw_bytes.size() >
        std::min(config_.max_response_bytes, kMaxDiagnosticResponseBytes))
      {
        result.reason = "diagnostic response exceeded maximum size";
        return complete_unresolved();
      }
      if (ch == '\r' || ch == '\n') {
        delimiter_observed = true;
        std::string line;
        if (!one_complete_line(result.raw_bytes, line)) {
          result.reason = "diagnostic response framing is ambiguous";
          return complete_unresolved();
        }
        result.response = line;
        const auto line_kind = classify_line(
          result.response, transaction.command, transaction.expected_prefix);
        if (line_kind != RoboteqLineKind::expected_reply || !valid_diagnostic_response(
            transaction.command, transaction.expected_prefix, result.response))
        {
          result.reason = line_kind == RoboteqLineKind::acknowledgement ?
            "unowned command acknowledgement encountered during diagnostic query" :
            "diagnostic response prefix or payload is malformed";
          return complete_unresolved();
        }
        result.delimiter_observed = true;
        observe_diagnostic(
          transaction, DiagnosticPhase::response_complete, result.raw_bytes.size());
        while (std::chrono::steady_clock::now() < deadline) {
          try {
            if (serial_.available() == 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
              continue;
            }
            char extra = 0;
            if (serial_.read(reinterpret_cast<uint8_t *>(&extra), 1) != 0) {
              result.raw_bytes.push_back(extra);
              result.last_byte_at = std::chrono::steady_clock::now();
              result.reason = "extra bytes after diagnostic response";
              return complete_unresolved();
            }
          } catch (const std::exception & ex) {
            result.reason = ex.what();
            return complete_unresolved();
          }
        }
        result.status = DiagnosticTransportStatus::success;
        result.completed_at = std::chrono::steady_clock::now();
        observe_diagnostic(
          transaction, DiagnosticPhase::transaction_complete, result.raw_bytes.size());
        return result;
      }
    } catch (const std::exception & ex) {
      result.reason = ex.what();
      return complete_unresolved();
    }
  }
  result.status = DiagnosticTransportStatus::timeout;
  result.reason = result.raw_bytes.empty() ? "diagnostic query timed out" :
    "diagnostic query timed out with a partial response";
  return complete_unresolved();
}

bool RoboteqSerialTransport::readRawUntilQuiet(
  const std::chrono::steady_clock::time_point & not_before,
  const std::chrono::steady_clock::time_point & absolute_deadline,
  std::chrono::milliseconds quiet_period,
  std::size_t max_bytes,
  std::string & raw,
  std::chrono::steady_clock::time_point & last_byte_at,
  bool & delimiter_observed,
  std::string & error)
{
  raw.clear();
  last_byte_at = std::chrono::steady_clock::time_point{};
  delimiter_observed = false;
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
      last_byte_at = std::chrono::steady_clock::now();
      if (raw.size() > max_bytes) {
        error = "bounded diagnostic drain exceeded maximum size";
        return false;
      }
      if (ch == '\r' || ch == '\n') {
        delimiter_observed = true;
      }
      quiet_deadline = std::chrono::steady_clock::now() + quiet_period;
    } catch (const std::exception & ex) {
      error = ex.what();
      return false;
    }
  }
  if (quiet_deadline <= absolute_deadline &&
    std::chrono::steady_clock::now() >= quiet_deadline)
  {
    return true;
  }
  error = "bounded diagnostic drain reached its absolute deadline";
  return false;
}

DiagnosticRecoveryResult RoboteqSerialTransport::boundedDiagnosticRecovery(
  const DiagnosticTransaction & timed_out_transaction,
  std::chrono::steady_clock::time_point timed_out_query_started_at,
  const DiagnosticTransaction & synchronization_transaction,
  const DiagnosticRecoveryBounds & bounds,
  const std::function<bool(std::string &)> & before_synchronization)
{
  DiagnosticRecoveryResult result;
  if (!is_allowed_diagnostic_command(timed_out_transaction.command) ||
    !is_allowed_diagnostic_command(synchronization_transaction.command) ||
    timed_out_transaction.expected_prefix == synchronization_transaction.expected_prefix)
  {
    result.reason = "invalid or non-distinguishable diagnostic recovery query";
    result.completed_at = std::chrono::steady_clock::now();
    return result;
  }

  std::string drain_error;
  result.drain_started_at = std::chrono::steady_clock::now();
  observe_diagnostic(timed_out_transaction, DiagnosticPhase::drain_started);
  if (!readRawUntilQuiet(
      timed_out_query_started_at + bounds.delayed_reply_horizon,
      timed_out_query_started_at + bounds.drain_absolute_limit,
      bounds.drain_quiet_period,
      bounds.max_drain_bytes,
      result.drained_raw_bytes,
      result.drain_last_byte_at,
      result.drain_delimiter_observed,
      drain_error))
  {
    result.reason = drain_error;
    result.drain_completed_at = std::chrono::steady_clock::now();
    observe_diagnostic(
      timed_out_transaction, DiagnosticPhase::drain_completed,
      result.drained_raw_bytes.size());
    result.completed_at = result.drain_completed_at;
    return result;
  }
  result.drain_completed_at = std::chrono::steady_clock::now();
  observe_diagnostic(
    timed_out_transaction, DiagnosticPhase::drain_completed,
    result.drained_raw_bytes.size());
  if (!result.drained_raw_bytes.empty()) {
    std::string delayed_line;
    if (!one_complete_line(result.drained_raw_bytes, delayed_line) ||
      !valid_diagnostic_response(
        timed_out_transaction.command, timed_out_transaction.expected_prefix, delayed_line))
    {
      result.reason = "drained delayed response is partial, malformed, or ambiguous";
      result.completed_at = std::chrono::steady_clock::now();
      return result;
    }
  }

  observe_diagnostic(timed_out_transaction, DiagnosticPhase::before_synchronization);
  if (before_synchronization && !before_synchronization(result.reason)) {
    if (result.reason.empty()) {
      result.reason = "pre-synchronization checkpoint failed";
    }
    result.completed_at = std::chrono::steady_clock::now();
    return result;
  }

  DiagnosticTransaction sync = synchronization_transaction;
  sync.timeout = bounds.synchronization_timeout;
  result.synchronization_started_at = std::chrono::steady_clock::now();
  observe_diagnostic(sync, DiagnosticPhase::waiting_for_synchronization);
  const auto sync_result = diagnosticQuery(sync);
  result.synchronization_response = sync_result.response;
  result.synchronization_last_byte_at = sync_result.last_byte_at;
  result.synchronization_delimiter_observed = sync_result.delimiter_observed;
  if (sync_result.status != DiagnosticTransportStatus::success) {
    result.drained_raw_bytes += sync_result.raw_bytes;
    result.reason = "synchronization query failed: " + sync_result.reason;
    result.completed_at = std::chrono::steady_clock::now();
    return result;
  }

  std::string extra_raw;
  if (!readRawUntilQuiet(
      std::chrono::steady_clock::now(),
      std::chrono::steady_clock::now() + bounds.post_sync_absolute_limit,
      bounds.post_sync_quiet_period,
      bounds.max_response_bytes,
      extra_raw,
      result.synchronization_last_byte_at,
      result.synchronization_delimiter_observed,
      drain_error))
  {
    result.drained_raw_bytes += extra_raw;
    result.reason = "post-synchronization framing check failed: " + drain_error;
    result.completed_at = std::chrono::steady_clock::now();
    return result;
  }
  if (!extra_raw.empty()) {
    result.drained_raw_bytes += extra_raw;
    result.reason = "extra bytes after synchronization response";
    result.completed_at = std::chrono::steady_clock::now();
    return result;
  }
  result.synchronized = true;
  result.completed_at = std::chrono::steady_clock::now();
  observe_diagnostic(sync, DiagnosticPhase::synchronization_complete);
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
