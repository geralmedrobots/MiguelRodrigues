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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace roboteq_ros2_driver
{

struct SerialTransportConfig
{
  std::string port{"/dev/roboteq"};
  int baud{115200};
  std::chrono::milliseconds read_timeout{50};
  std::chrono::milliseconds write_timeout{50};
  std::chrono::milliseconds transaction_timeout{100};
  std::size_t max_response_bytes{256};
};

enum class DiagnosticTransportStatus
{
  success,
  timeout,
  failure,
};

struct DiagnosticTransaction
{
  std::string command;
  std::string expected_prefix;
  std::chrono::milliseconds timeout{100};
};

struct DiagnosticTransactionResult
{
  DiagnosticTransportStatus status{DiagnosticTransportStatus::failure};
  std::string response;
  std::string raw_bytes;
  std::string reason;
  std::chrono::steady_clock::time_point started_at{};
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
};

class IRoboteqSerialTransport
{
public:
  virtual ~IRoboteqSerialTransport() = default;

  virtual bool open(std::string & error) = 0;
  virtual void close() noexcept = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual bool sendCommands(const std::vector<std::string> & commands, std::string & error) = 0;
  virtual bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) = 0;
  virtual DiagnosticTransactionResult diagnosticQuery(
    const DiagnosticTransaction & transaction) = 0;
  virtual DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const DiagnosticTransaction & timed_out_transaction,
    std::chrono::steady_clock::time_point timed_out_query_started_at,
    const DiagnosticTransaction & synchronization_transaction,
    const DiagnosticRecoveryBounds & bounds) = 0;
};

class RoboteqSerialTransport : public IRoboteqSerialTransport
{
public:
  explicit RoboteqSerialTransport(SerialTransportConfig config);

  bool open(std::string & error) override;
  void close() noexcept override;
  bool isOpen() const noexcept override;
  bool sendCommands(const std::vector<std::string> & commands, std::string & error) override;
  bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) override;
  DiagnosticTransactionResult diagnosticQuery(
    const DiagnosticTransaction & transaction) override;
  DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const DiagnosticTransaction & timed_out_transaction,
    std::chrono::steady_clock::time_point timed_out_query_started_at,
    const DiagnosticTransaction & synchronization_transaction,
    const DiagnosticRecoveryBounds & bounds) override;

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
    std::string & error);

  SerialTransportConfig config_;
  serial::Serial serial_;
};

std::string strip_roboteq_line_endings(const std::string & text);

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_
