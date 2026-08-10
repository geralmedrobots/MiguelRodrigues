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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_

#include <serial/serial.h>

#include <cstddef>
#include <cstdint>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace roboteq_ros2_driver
{

enum class DiagnosticTransportStatus
{
  success,
  timeout,
  failure,
};

enum class QueryLineClassification
{
  none,
  acknowledgement,
  rejection,
  echo,
  expected_reply,
  unexpected_reply,
};

struct QueryTraceEvent
{
  std::string command;
  std::string expected_prefix;
  std::string response;
  std::string raw_bytes;
  std::string reason;
  QueryLineClassification classification{QueryLineClassification::none};
  bool success{false};
  bool delimiter_observed{false};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point write_started_at{};
  std::chrono::steady_clock::time_point write_accepted_at{};
  std::chrono::steady_clock::time_point first_byte_at{};
  std::chrono::steady_clock::time_point last_byte_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

struct SerialTransportConfig
{
  std::string port{"/dev/roboteq"};
  int baud{115200};
  std::chrono::milliseconds read_timeout{50};
  std::chrono::milliseconds write_timeout{50};
  std::chrono::milliseconds transaction_timeout{100};
  std::chrono::milliseconds post_reply_quiet_period{20};
  std::size_t max_response_bytes{256};
  std::function<void(const QueryTraceEvent &)> query_observer;
};

struct DiagnosticTransaction
{
  DiagnosticTransaction() = default;
  DiagnosticTransaction(
    std::string command_value,
    std::string expected_prefix_value,
    std::chrono::milliseconds timeout_value)
  : command(std::move(command_value)),
    expected_prefix(std::move(expected_prefix_value)),
    timeout(timeout_value)
  {}

  std::string command;
  std::string expected_prefix;
  std::chrono::milliseconds timeout{100};
  uint64_t correlation_id{0};
  uint64_t connection_generation{0};
  std::function<void(const struct DiagnosticPhaseEvent &)> observer;
};

enum class DiagnosticPhase
{
  selected,
  write_started,
  write_accepted,
  waiting_for_first_byte,
  first_byte_received,
  response_complete,
  transaction_complete,
  timeout_or_unresolved,
  drain_started,
  drain_completed,
  before_synchronization,
  waiting_for_synchronization,
  synchronization_complete,
  before_fallback_close,
  reconnect_complete,
};

struct DiagnosticPhaseEvent
{
  DiagnosticPhase phase{DiagnosticPhase::write_started};
  std::chrono::steady_clock::time_point timestamp{};
  uint64_t correlation_id{0};
  uint64_t connection_generation{0};
  std::string command;
  std::size_t byte_count{0};
};

struct DiagnosticTransactionResult
{
  DiagnosticTransportStatus status{DiagnosticTransportStatus::failure};
  std::string response;
  std::string raw_bytes;
  std::string reason;
  bool delimiter_observed{false};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point write_accepted_at{};
  std::chrono::steady_clock::time_point first_byte_at{};
  std::chrono::steady_clock::time_point last_byte_at{};
  std::chrono::steady_clock::time_point timeout_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

struct DiagnosticRecoveryBounds
{
  std::chrono::milliseconds delayed_reply_horizon{100};
  std::chrono::milliseconds drain_absolute_limit{120};
  std::chrono::milliseconds drain_quiet_period{20};
  std::size_t max_drain_bytes{4096};
  std::chrono::milliseconds synchronization_timeout{100};
  std::chrono::milliseconds post_sync_absolute_limit{50};
  std::chrono::milliseconds post_sync_quiet_period{20};
  std::size_t max_response_bytes{256};
};

struct DiagnosticRecoveryResult
{
  bool synchronized{false};
  std::string drained_raw_bytes;
  std::string synchronization_response;
  std::string reason;
  bool drain_delimiter_observed{false};
  bool synchronization_delimiter_observed{false};
  std::chrono::steady_clock::time_point drain_started_at{};
  std::chrono::steady_clock::time_point drain_completed_at{};
  std::chrono::steady_clock::time_point drain_last_byte_at{};
  std::chrono::steady_clock::time_point synchronization_started_at{};
  std::chrono::steady_clock::time_point synchronization_last_byte_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

enum class CommandTransportStatus
{
  success,
  unresolved,
  failure,
};

struct CommandTransactionBounds
{
  std::chrono::milliseconds acknowledgement_deadline{50};
  std::chrono::milliseconds post_ack_quiet_period{20};
  std::size_t max_response_bytes{64};
};

struct CommandTransactionResult
{
  CommandTransportStatus status{CommandTransportStatus::failure};
  std::string raw_bytes;
  std::string reason;
  std::size_t expected_acknowledgements{0};
  std::size_t received_acknowledgements{0};
  bool partial_line{false};
  bool write_fully_accepted{false};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point write_accepted_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

struct StartupDrainBounds
{
  std::chrono::milliseconds absolute_limit{250};
  std::chrono::milliseconds quiet_period{100};
  std::size_t max_bytes{256};
};

struct StartupDrainResult
{
  bool synchronized{false};
  std::string raw_bytes;
  std::string reason;
  bool delimiter_observed{false};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point last_byte_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

class IRoboteqSerialTransport
{
public:
  virtual ~IRoboteqSerialTransport() = default;

  virtual bool open(std::string & error) = 0;
  virtual void close() noexcept = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual StartupDrainResult drainStartupInput(const StartupDrainBounds & bounds) = 0;
  virtual CommandTransactionResult commandTransaction(
    const std::vector<std::string> & commands,
    const CommandTransactionBounds & bounds) = 0;
  virtual bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) = 0;
  virtual bool queryWithTimeout(
    const std::string & command,
    const std::string & expected_prefix,
    std::chrono::milliseconds timeout,
    std::string & response,
    std::string & error)
  {
    (void)timeout;
    return query(command, expected_prefix, response, error);
  }
  virtual DiagnosticTransactionResult diagnosticQuery(
    const DiagnosticTransaction & transaction) = 0;
  virtual DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const DiagnosticTransaction & timed_out_transaction,
    std::chrono::steady_clock::time_point timed_out_query_started_at,
    const DiagnosticTransaction & synchronization_transaction,
    const DiagnosticRecoveryBounds & bounds,
    const std::function<bool(std::string &)> & before_synchronization) = 0;
};

class RoboteqSerialTransport : public IRoboteqSerialTransport
{
public:
  explicit RoboteqSerialTransport(SerialTransportConfig config);

  bool open(std::string & error) override;
  void close() noexcept override;
  bool isOpen() const noexcept override;
  StartupDrainResult drainStartupInput(const StartupDrainBounds & bounds) override;
  CommandTransactionResult commandTransaction(
    const std::vector<std::string> & commands,
    const CommandTransactionBounds & bounds) override;
  bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) override;
  bool queryWithTimeout(
    const std::string & command,
    const std::string & expected_prefix,
    std::chrono::milliseconds timeout,
    std::string & response,
    std::string & error) override;
  DiagnosticTransactionResult diagnosticQuery(
    const DiagnosticTransaction & transaction) override;
  DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const DiagnosticTransaction & timed_out_transaction,
    std::chrono::steady_clock::time_point timed_out_query_started_at,
    const DiagnosticTransaction & synchronization_transaction,
    const DiagnosticRecoveryBounds & bounds,
    const std::function<bool(std::string &)> & before_synchronization) override;

private:
  bool readLine(
    const std::chrono::steady_clock::time_point & deadline,
    std::string & line,
    std::string & error);
  bool writeReadOnlyDiagnostic(const std::string & command, std::string & error);
  bool readRawUntilQuiet(
    const std::chrono::steady_clock::time_point & not_before,
    const std::chrono::steady_clock::time_point & absolute_deadline,
    std::chrono::milliseconds quiet_period,
    std::size_t max_bytes,
    std::string & raw,
    std::chrono::steady_clock::time_point & last_byte_at,
    bool & delimiter_observed,
    std::string & error);

  SerialTransportConfig config_;
  serial::Serial serial_;
};

std::string strip_roboteq_line_endings(const std::string & text);

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_
