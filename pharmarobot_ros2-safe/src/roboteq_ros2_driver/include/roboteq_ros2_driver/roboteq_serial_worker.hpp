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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_WORKER_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_WORKER_HPP_

#include <optional>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "roboteq_ros2_driver/roboteq_configuration.hpp"
#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"

namespace roboteq_ros2_driver
{

struct DesiredMotorCommand
{
  double channel_1_mps{0.0};
  double channel_2_mps{0.0};
  std::chrono::steady_clock::time_point received_time{};
  uint64_t sequence{0};
  bool valid{false};
};

struct EncoderSample
{
  int32_t channel_1{0};
  int32_t channel_2{0};
  std::chrono::steady_clock::time_point timestamp{};
  uint64_t sequence{0};
  bool valid{false};
};

enum class DiagnosticQueryKind
{
  firmware_id,
  fault_flags,
  motor_status_1,
  motor_status_2,
  status_flags,
};

enum class SerialFramingState
{
  synchronized,
  unresolved,
};

struct DiagnosticTelemetry
{
  DiagnosticQueryKind query{DiagnosticQueryKind::firmware_id};
  std::string raw_value;
  bool valid{false};
  std::chrono::steady_clock::time_point timestamp{};
  std::chrono::milliseconds age{0};
  uint64_t connection_generation{0};
  uint64_t correlation_id{0};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point write_accepted_at{};
  std::chrono::steady_clock::time_point first_byte_at{};
  std::chrono::steady_clock::time_point last_byte_at{};
  std::chrono::steady_clock::time_point timeout_at{};
  bool delimiter_observed{false};
  std::string failure_reason{"not sampled"};
};

struct DiagnosticResultEvent
{
  DiagnosticQueryKind query{DiagnosticQueryKind::firmware_id};
  DiagnosticTransportStatus status{DiagnosticTransportStatus::failure};
  SerialFramingState framing_state{SerialFramingState::synchronized};
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
  uint64_t connection_generation{0};
  uint64_t correlation_id{0};
};

enum class SerialConnectionState
{
  disconnected,
  connecting,
  configuring,
  waiting_for_fresh_command,
  ready,
  unhealthy,
  reconnecting,
};

struct SerialWorkerStatus
{
  SerialConnectionState connection_state{SerialConnectionState::disconnected};
  bool transport_open{false};
  bool ready_for_motion{false};
  bool have_encoder_sample{false};
  bool require_fresh_command_after_reconnect{true};
  std::chrono::steady_clock::time_point latest_encoder_timestamp{};
  uint64_t latest_encoder_sequence{0};
  uint64_t command_sequence{0};
  uint64_t update_sequence{0};
  uint64_t connection_generation{0};
  SerialFramingState framing_state{SerialFramingState::synchronized};
  bool diagnostic_recovery_pending{false};
};

enum class TimeoutStopEventPhase
{
  timeout_detected,
  zero_write_started,
  zero_write_completed,
};

struct TimeoutStopEvent
{
  TimeoutStopEventPhase phase{TimeoutStopEventPhase::timeout_detected};
  std::chrono::steady_clock::time_point timestamp{};
  uint64_t command_sequence{0};
  bool write_succeeded{false};
};

enum class StopRequestPhase
{
  requested,
  coalesced,
  write_started,
  write_accepted,
  write_failed,
};

struct StopRequestEvent
{
  StopRequestPhase phase{StopRequestPhase::requested};
  std::chrono::steady_clock::time_point timestamp{};
  uint64_t correlation_id{0};
  uint64_t connection_generation{0};
  std::size_t byte_count{0};
};

struct SerialWorkerConfig
{
  bool open_loop{false};
  double wheel_circumference{1.0};
  int max_rpm{100};
  std::chrono::milliseconds command_timeout{500};
  std::chrono::milliseconds encoder_poll_period{50};
  std::chrono::milliseconds reconnect_interval{1000};
  std::chrono::milliseconds diagnostic_query_timeout{100};
  StartupDrainBounds startup_drain_bounds{};
  CommandTransactionBounds command_transaction_bounds{};
  DiagnosticRecoveryBounds diagnostic_recovery_bounds{};
  bool require_fresh_command_after_reconnect{true};
  std::vector<configuration::RequiredControllerSetting> required_settings;
  std::function<void(const std::string &)> log_callback;
  std::function<void(const TimeoutStopEvent &)> timeout_stop_observer;
  std::function<void(const StopRequestEvent &)> stop_request_observer;
  std::function<void(const DiagnosticPhaseEvent &)> diagnostic_phase_observer;
  std::function<void(const DiagnosticResultEvent &)> diagnostic_result_observer;
};

class SerialIoWorker
{
public:
  SerialIoWorker(
    std::unique_ptr<IRoboteqSerialTransport> transport,
    SerialWorkerConfig config);
  ~SerialIoWorker();

  SerialIoWorker(const SerialIoWorker &) = delete;
  SerialIoWorker & operator=(const SerialIoWorker &) = delete;

  void start();
  void stop();
  uint64_t requestStop();
  void submitCommand(double channel_1_mps, double channel_2_mps);
  void invalidateCommands();
  std::optional<EncoderSample> takeLatestEncoderSample();
  uint64_t commandSequence() const;
  bool isConnected() const;
  bool isReadyForMotion() const;
  SerialWorkerStatus status() const;
  bool queueDiagnosticQuery(DiagnosticQueryKind query);
  std::optional<DiagnosticTelemetry> latestDiagnosticTelemetry() const;

private:
  enum class DiagnosticRecoveryAttempt
  {
    completed,
    deferred,
    failed,
  };

  void run();
  bool connectAndValidate(std::string & error);
  bool sendStop(
    const char * reason, std::string & error,
    std::chrono::steady_clock::time_point * write_accepted_at = nullptr,
    bool * write_fully_accepted = nullptr);
  bool sendOwnedCommands(
    const std::vector<std::string> & commands, std::string & error,
    std::chrono::steady_clock::time_point * write_accepted_at = nullptr,
    bool * write_fully_accepted = nullptr);
  void scheduleOwnershipRecovery(std::chrono::steady_clock::time_point started_at);
  bool executePendingRuntimeStop(std::string & error);
  void observeStopRequest(const StopRequestEvent & event) const noexcept;
  void observeDiagnosticPhase(const DiagnosticPhaseEvent & event) const noexcept;
  void observeTimeoutStop(const TimeoutStopEvent & event) const noexcept;
  bool sendDesiredCommand(const DesiredMotorCommand & command, std::string & error);
  bool validateControllerConfiguration(std::string & error);
  bool validateCommunication(std::string & error);
  bool pollEncoder(std::string & error);
  DiagnosticRecoveryAttempt performDiagnosticRecovery(std::string & error);
  bool executeDiagnosticQuery(DiagnosticQueryKind query);
  void invalidateDiagnosticTelemetry(const std::string & reason);
  void markFailure(const std::string & error);
  std::vector<std::string> buildMotorCommands(double channel_1_mps, double channel_2_mps) const;
  std::chrono::steady_clock::time_point nextWakeTime(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point next_encoder_poll,
    std::chrono::steady_clock::time_point next_reconnect) const;

  std::unique_ptr<IRoboteqSerialTransport> transport_;
  SerialWorkerConfig config_;

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  DesiredMotorCommand desired_command_;
  uint64_t latest_submitted_sequence_{0};
  uint64_t applied_sequence_{0};
  uint64_t minimum_motion_sequence_{0};
  bool applied_stopped_{true};
  bool transport_open_{false};
  bool last_command_transaction_owned_{false};  // Accessed only by the worker thread.
  bool stop_requested_{false};
  bool runtime_stop_pending_{false};
  uint64_t runtime_stop_correlation_{0};
  uint64_t next_validation_correlation_{0};
  uint64_t last_recovery_correlation_{0};
  std::chrono::steady_clock::time_point runtime_stop_requested_at_{};
  bool worker_started_{false};
  std::optional<EncoderSample> latest_encoder_sample_;
  std::optional<EncoderSample> last_encoder_sample_;
  uint64_t encoder_sequence_{0};
  uint64_t status_update_sequence_{0};
  uint64_t connection_generation_{0};
  SerialFramingState framing_state_{SerialFramingState::synchronized};
  std::optional<DiagnosticQueryKind> queued_diagnostic_query_;
  std::optional<DiagnosticTransaction> timed_out_diagnostic_;
  std::chrono::steady_clock::time_point timed_out_diagnostic_started_at_{};
  bool diagnostic_recovery_pending_{false};
  bool diagnostic_recovery_start_reserved_{false};
  std::optional<DiagnosticTelemetry> latest_diagnostic_telemetry_;
  SerialConnectionState state_{SerialConnectionState::disconnected};
  std::thread worker_thread_;
};

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_WORKER_HPP_
