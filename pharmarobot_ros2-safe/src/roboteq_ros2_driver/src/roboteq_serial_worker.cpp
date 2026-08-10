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

#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

#include <charconv>
#include <string_view>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "roboteq_ros2_driver/command_scaling.hpp"
#include "roboteq_ros2_driver/roboteq_protocol.hpp"

namespace roboteq_ros2_driver
{
namespace
{

std::string query_for_setting(const configuration::RequiredControllerSetting & setting)
{
  std::ostringstream query;
  query << "~" << setting.name;
  if (setting.channel > 0) {
    query << " " << setting.channel;
  }
  query << "\r";
  return query.str();
}

std::string visible_text(const std::string & text)
{
  if (text.empty()) {
    return "<no response>";
  }
  std::string visible;
  for (const unsigned char ch : text) {
    if (ch == '\r') {
      visible += "\\r";
    } else if (ch == '\n') {
      visible += "\\n";
    } else if (ch == '\t') {
      visible += "\\t";
    } else if (ch == '\\') {
      visible += "\\\\";
    } else if (ch == '"') {
      visible += "\\\"";
    } else if (ch >= 0x20 && ch <= 0x7e) {
      visible += static_cast<char>(ch);
    } else {
      std::ostringstream escaped;
      escaped << "\\x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') <<
        static_cast<int>(ch);
      visible += escaped.str();
    }
  }
  return visible;
}

std::string validation_context(
  const std::string & phase,
  const std::string & category,
  const std::string & name,
  int channel,
  const std::string & query,
  const std::string & expected_prefix,
  const std::string & expected_value,
  const std::string & response,
  const std::string & reason)
{
  std::ostringstream stream;
  stream << "phase=" << phase << " category=" << category << " query_name=" << name <<
    " channel=";
  if (channel > 0) {
    stream << channel;
  } else {
    stream << "none";
  }
  stream << " transmitted=\"" << visible_text(query) <<
    "\" expected_prefix=\"" << expected_prefix <<
    "\" expected_value=\"" << expected_value <<
    "\" received=\"" << visible_text(response) <<
    "\" reason=" << visible_text(reason);
  return stream.str();
}

std::string transport_failure_category(const std::string & error)
{
  if (error.find("rejected") != std::string::npos) {
    return "rejection";
  }
  if (error.find("unexpected response prefix") != std::string::npos) {
    return "wrong_prefix";
  }
  if (error.find("timed out") != std::string::npos) {
    return "timeout";
  }
  return "transport_error";
}

std::string malformed_numeric_reason(
  const std::string & response, const std::string & expected_prefix)
{
  const std::string_view payload(response.data() + expected_prefix.size(),
    response.size() - expected_prefix.size());
  if (payload.empty()) {
    return "malformed numeric response: value is empty";
  }
  int ignored = 0;
  const auto parsed = std::from_chars(payload.data(), payload.data() + payload.size(), ignored);
  if (parsed.ec == std::errc::invalid_argument) {
    return "malformed numeric response: value is not an integer";
  }
  if (parsed.ec == std::errc::result_out_of_range) {
    return "malformed numeric response: integer is out of range";
  }
  return "malformed numeric response: trailing characters after integer";
}

DiagnosticTransaction diagnostic_transaction(
  DiagnosticQueryKind query, std::chrono::milliseconds timeout)
{
  switch (query) {
    case DiagnosticQueryKind::firmware_id:
      return {"?FID\r", "FID=", timeout};
    case DiagnosticQueryKind::fault_flags:
      return {"?FF\r", "FF=", timeout};
    case DiagnosticQueryKind::motor_status_1:
      return {"?FM 1\r", "FM=", timeout};
    case DiagnosticQueryKind::motor_status_2:
      return {"?FM 2\r", "FM=", timeout};
    case DiagnosticQueryKind::status_flags:
      return {"?FS\r", "FS=", timeout};
  }
  return {};
}

DiagnosticTransaction synchronization_transaction(
  const DiagnosticTransaction & timed_out, std::chrono::milliseconds timeout)
{
  if (timed_out.command == "?FF\r") {
    return {"?FS\r", "FS=", timeout};
  }
  return {"?FF\r", "FF=", timeout};
}

}  // namespace

SerialIoWorker::SerialIoWorker(
  std::unique_ptr<IRoboteqSerialTransport> transport,
  SerialWorkerConfig config)
: transport_(std::move(transport)),
  config_(std::move(config))
{
}

SerialIoWorker::~SerialIoWorker()
{
  stop();
}

void SerialIoWorker::start()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (worker_started_) {
    return;
  }
  stop_requested_ = false;
  worker_started_ = true;
  worker_thread_ = std::thread(&SerialIoWorker::run, this);
}

void SerialIoWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!worker_started_) {
      return;
    }
    stop_requested_ = true;
    desired_command_.valid = false;
    minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
  }
  state_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    worker_started_ = false;
  }
}

uint64_t SerialIoWorker::requestStop()
{
  StopRequestEvent event;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_command_.valid = false;
    minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
    diagnostic_recovery_start_reserved_ = false;
    if (runtime_stop_pending_) {
      event = StopRequestEvent{
        StopRequestPhase::coalesced, std::chrono::steady_clock::now(),
        runtime_stop_correlation_, connection_generation_, 0};
    } else {
      runtime_stop_pending_ = true;
      runtime_stop_correlation_ = ++next_validation_correlation_;
      runtime_stop_requested_at_ = std::chrono::steady_clock::now();
      event = StopRequestEvent{
        StopRequestPhase::requested, runtime_stop_requested_at_,
        runtime_stop_correlation_, connection_generation_, 0};
    }
  }
  observeStopRequest(event);
  state_cv_.notify_all();
  return event.correlation_id;
}

void SerialIoWorker::submitCommand(double channel_1_mps, double channel_2_mps)
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_submitted_sequence_++;
    desired_command_ = DesiredMotorCommand{
      channel_1_mps,
      channel_2_mps,
      now,
      latest_submitted_sequence_,
      true};
  }
  state_cv_.notify_all();
}

void SerialIoWorker::invalidateCommands()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  desired_command_.valid = false;
  minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
  applied_stopped_ = true;
}

std::optional<EncoderSample> SerialIoWorker::takeLatestEncoderSample()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto sample = latest_encoder_sample_;
  if (sample.has_value()) {
    last_encoder_sample_ = sample;
  }
  latest_encoder_sample_.reset();
  return sample;
}

uint64_t SerialIoWorker::commandSequence() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_submitted_sequence_;
}

bool SerialIoWorker::isConnected() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return transport_open_;
}

bool SerialIoWorker::isReadyForMotion() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_ == SerialConnectionState::ready &&
         framing_state_ == SerialFramingState::synchronized;
}

SerialWorkerStatus SerialIoWorker::status() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return SerialWorkerStatus{
    state_,
    transport_open_,
    state_ == SerialConnectionState::ready &&
    framing_state_ == SerialFramingState::synchronized,
    latest_encoder_sample_.has_value() || last_encoder_sample_.has_value(),
    config_.require_fresh_command_after_reconnect,
    latest_encoder_sample_ ? latest_encoder_sample_->timestamp :
    (last_encoder_sample_ ? last_encoder_sample_->timestamp :
    std::chrono::steady_clock::time_point{}),
    latest_encoder_sample_ ? latest_encoder_sample_->sequence :
    (last_encoder_sample_ ? last_encoder_sample_->sequence : 0),
    latest_submitted_sequence_,
    status_update_sequence_,
    connection_generation_,
    framing_state_,
    diagnostic_recovery_pending_,
  };
}

bool SerialIoWorker::queueDiagnosticQuery(DiagnosticQueryKind query)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (queued_diagnostic_query_.has_value() || diagnostic_recovery_pending_) {
    return false;
  }
  queued_diagnostic_query_ = query;
  state_cv_.notify_all();
  return true;
}

std::optional<DiagnosticTelemetry> SerialIoWorker::latestDiagnosticTelemetry() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto telemetry = latest_diagnostic_telemetry_;
  if (telemetry.has_value() && telemetry->timestamp != std::chrono::steady_clock::time_point{}) {
    telemetry->age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - telemetry->timestamp);
    if (telemetry->connection_generation != connection_generation_) {
      telemetry->valid = false;
      if (!telemetry->failure_reason.empty()) {
        telemetry->failure_reason += "; ";
      }
      telemetry->failure_reason += "telemetry belongs to an old connection generation";
    }
  }
  return telemetry;
}

std::optional<MotorTelemetrySnapshot> SerialIoWorker::latestMotorTelemetry() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto telemetry = latest_motor_telemetry_;
  if (!telemetry.has_value()) {
    return telemetry;
  }
  telemetry->age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - telemetry->timestamp);
  telemetry->channel_1.age = telemetry->age;
  telemetry->channel_2.age = telemetry->age;
  if (telemetry->age > config_.telemetry_stale_after) {
    telemetry->valid = false;
    telemetry->failure_reason = "telemetry is stale";
    telemetry->channel_1.valid = false;
    telemetry->channel_2.valid = false;
  }
  if (telemetry->connection_generation != connection_generation_) {
    // Snapshot generation is represented by the worker connection sequence in
    // the sample sequence; a reconnect invalidates all previous samples.
    telemetry->valid = false;
    telemetry->channel_1.valid = false;
    telemetry->channel_2.valid = false;
    telemetry->failure_reason = "telemetry belongs to an old connection generation";
  }
  return telemetry;
}

void SerialIoWorker::run()
{
  auto next_encoder_poll = std::chrono::steady_clock::now();
  auto next_reconnect = std::chrono::steady_clock::now();
  auto next_telemetry_poll = std::chrono::steady_clock::now();

  while (true) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      if (stop_requested_) {
        break;
      }
    }

    if (!transport_->isOpen()) {
      const auto now = std::chrono::steady_clock::now();
      if (now < next_reconnect) {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait_until(lock, next_reconnect, [this]() {return stop_requested_;});
        continue;
      }

      std::string error;
      if (!connectAndValidate(error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        minimum_motion_sequence_ = config_.require_fresh_command_after_reconnect ?
          latest_submitted_sequence_ + 1 : minimum_motion_sequence_;
        applied_sequence_ = 0;
        applied_stopped_ = true;
        state_ = config_.require_fresh_command_after_reconnect ?
          SerialConnectionState::waiting_for_fresh_command : SerialConnectionState::ready;
        status_update_sequence_++;
      }
      uint64_t reconnect_correlation = 0;
      uint64_t reconnect_generation = 0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        reconnect_correlation = last_recovery_correlation_;
        reconnect_generation = connection_generation_;
        last_recovery_correlation_ = 0;
      }
      if (reconnect_correlation != 0) {
        observeDiagnosticPhase(
          DiagnosticPhaseEvent{
            DiagnosticPhase::reconnect_complete, std::chrono::steady_clock::now(),
            reconnect_correlation, reconnect_generation, "", 0});
      }
      next_encoder_poll = std::chrono::steady_clock::now() + config_.encoder_poll_period;
      next_telemetry_poll = std::chrono::steady_clock::now() + config_.telemetry_poll_period;
      next_reconnect = std::chrono::steady_clock::time_point::max();
    }

    std::string priority_stop_error;
    if (!executePendingRuntimeStop(priority_stop_error)) {
      markFailure(priority_stop_error);
      next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
      continue;
    }

    DesiredMotorCommand command;
    bool have_command = false;
    bool timeout_stop_required = false;
    uint64_t timed_out_command_sequence = 0;
    std::chrono::steady_clock::time_point timeout_detected_at;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      command = desired_command_;
      have_command = command.valid &&
        command.sequence > applied_sequence_ &&
        command.sequence >= minimum_motion_sequence_;

      if (desired_command_.valid && !applied_stopped_) {
        const auto timeout_check_at = std::chrono::steady_clock::now();
        const auto age = timeout_check_at - desired_command_.received_time;
        timeout_stop_required = age >= config_.command_timeout;
        if (timeout_stop_required) {
          timed_out_command_sequence = desired_command_.sequence;
          timeout_detected_at = timeout_check_at;
        }
      }
    }

    std::string error;
    const bool command_permitted =
      have_command && framing_state_ == SerialFramingState::synchronized;
    if (timeout_stop_required) {
      observeTimeoutStop(
        TimeoutStopEvent{
          TimeoutStopEventPhase::timeout_detected,
          timeout_detected_at,
          timed_out_command_sequence,
          false});
      observeTimeoutStop(
        TimeoutStopEvent{
          TimeoutStopEventPhase::zero_write_started,
          std::chrono::steady_clock::now(),
          timed_out_command_sequence,
          false});
      std::chrono::steady_clock::time_point stop_write_accepted_at;
      bool stop_write_fully_accepted = false;
      const bool stop_transaction_continues = sendStop(
        "command timeout", error, &stop_write_accepted_at, &stop_write_fully_accepted);
      observeTimeoutStop(
        TimeoutStopEvent{
          TimeoutStopEventPhase::zero_write_completed,
          stop_write_fully_accepted &&
          stop_write_accepted_at != std::chrono::steady_clock::time_point{} ?
          stop_write_accepted_at : std::chrono::steady_clock::now(),
          timed_out_command_sequence,
          stop_write_fully_accepted});
      if (!stop_transaction_continues) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied_stopped_ = true;
      applied_sequence_ = latest_submitted_sequence_;
      desired_command_.valid = false;
      minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
    } else if (command_permitted) {
      if (!sendDesiredCommand(command, error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (last_command_transaction_owned_) {
        applied_sequence_ = command.sequence;
        applied_stopped_ =
          std::abs(command.channel_1_mps) < 1e-12 && std::abs(command.channel_2_mps) < 1e-12;
        state_ = SerialConnectionState::ready;
      }
      status_update_sequence_++;
    }

    bool recovery_pending = false;
    bool recovery_can_start = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      recovery_pending = diagnostic_recovery_pending_;
      if (recovery_pending) {
        const auto recovery_claimed_at = std::chrono::steady_clock::now();
        const bool pending_runtime_stop_now = runtime_stop_pending_;
        const bool timeout_stop_now = desired_command_.valid && !applied_stopped_ &&
          recovery_claimed_at - desired_command_.received_time >= config_.command_timeout;
        recovery_can_start =
          !pending_runtime_stop_now && !timeout_stop_now;
        diagnostic_recovery_start_reserved_ = recovery_can_start;
      } else {
        diagnostic_recovery_start_reserved_ = false;
      }
    }
    if (recovery_pending && recovery_can_start) {
      const auto recovery_result = performDiagnosticRecovery(error);
      if (recovery_result == DiagnosticRecoveryAttempt::failed) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
      }
      continue;
    }
    if (recovery_pending) {
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_encoder_poll) {
      if (!pollEncoder(error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      next_encoder_poll = std::chrono::steady_clock::now() + config_.encoder_poll_period;
    }

    bool telemetry_allowed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool timeout_stop_now = desired_command_.valid && !applied_stopped_ &&
        std::chrono::steady_clock::now() - desired_command_.received_time >=
        config_.command_timeout;
      const bool pending_command_now = desired_command_.valid &&
        desired_command_.sequence > applied_sequence_ &&
        desired_command_.sequence >= minimum_motion_sequence_;
      telemetry_allowed = config_.telemetry_enabled &&
        framing_state_ == SerialFramingState::synchronized &&
        !timeout_stop_now && !pending_command_now &&
        std::chrono::steady_clock::now() >= next_telemetry_poll;
    }
    if (telemetry_allowed) {
      if (!pollMotorTelemetry(error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      // Continue an in-progress snapshot immediately, but return to the main loop
      // between every query so a pending command, timeout stop, or recovery always
      // wins over the next telemetry transaction.  Only a completed snapshot starts
      // the configured bounded-rate polling interval.
      next_telemetry_poll = telemetry_query_index_ == 0 ?
        std::chrono::steady_clock::now() + config_.telemetry_poll_period :
        std::chrono::steady_clock::now();
      continue;
    }


    std::optional<DiagnosticQueryKind> diagnostic_query;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto diagnostic_claimed_at = std::chrono::steady_clock::now();
      const bool timeout_stop_now = desired_command_.valid && !applied_stopped_ &&
        diagnostic_claimed_at - desired_command_.received_time >= config_.command_timeout;
      const bool pending_command_now = desired_command_.valid &&
        desired_command_.sequence > applied_sequence_ &&
        desired_command_.sequence >= minimum_motion_sequence_;
      if (framing_state_ == SerialFramingState::synchronized &&
        !timeout_stop_now && !pending_command_now && diagnostic_claimed_at < next_encoder_poll)
      {
        diagnostic_query = queued_diagnostic_query_;
        queued_diagnostic_query_.reset();
      }
    }
    if (diagnostic_query.has_value()) {
      if (!executeDiagnosticQuery(*diagnostic_query)) {
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
      }
      continue;
    }

    std::unique_lock<std::mutex> lock(state_mutex_);
    const auto next_feedback_wake = config_.telemetry_enabled ?
      std::min(next_encoder_poll, next_telemetry_poll) : next_encoder_poll;
    state_cv_.wait_until(
      lock,
      nextWakeTime(std::chrono::steady_clock::now(), next_feedback_wake, next_reconnect),
      [this, &next_telemetry_poll]() {
        return stop_requested_ ||
        runtime_stop_pending_ ||
        queued_diagnostic_query_.has_value() || diagnostic_recovery_pending_ ||
        (config_.telemetry_enabled &&
        std::chrono::steady_clock::now() >= next_telemetry_poll) ||
        (desired_command_.valid &&
        desired_command_.sequence > applied_sequence_ &&
        desired_command_.sequence >= minimum_motion_sequence_);
      });
  }

  std::string ignored_error;
  if (transport_->isOpen()) {
    sendStop("driver shutdown", ignored_error);
    transport_->close();
    std::lock_guard<std::mutex> lock(state_mutex_);
    transport_open_ = false;
    status_update_sequence_++;
  }
}

bool SerialIoWorker::connectAndValidate(std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = SerialConnectionState::connecting;
    status_update_sequence_++;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker connecting");
  }
  try {
    if (!transport_->open(error)) {
      error = "connectAndValidate: phase=connection category=transport_error "
        "operation=serial_open reason=" + visible_text(error);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      transport_open_ = true;
      connection_generation_++;
      framing_state_ = SerialFramingState::synchronized;
      diagnostic_recovery_pending_ = false;
      diagnostic_recovery_start_reserved_ = false;
      timed_out_diagnostic_.reset();
      queued_diagnostic_query_.reset();
      invalidateDiagnosticTelemetry("connection generation changed");
      latest_motor_telemetry_.reset();
      telemetry_query_index_ = 0;
      status_update_sequence_++;
    }
    const auto startup_drain = transport_->drainStartupInput(config_.startup_drain_bounds);
    if (!startup_drain.synchronized) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        framing_state_ = SerialFramingState::unresolved;
        status_update_sequence_++;
      }
      error = "connectAndValidate: phase=connection category=transport_error "
        "operation=startup_input_drain received=\"" +
        visible_text(startup_drain.raw_bytes) + "\" reason=" + visible_text(startup_drain.reason);
      return false;
    }
    if (!executePendingRuntimeStop(error)) {
      error = "connectAndValidate: phase=connection category=transport_error "
        "operation=pending_runtime_stop reason=" + visible_text(error);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (framing_state_ != SerialFramingState::synchronized ||
        diagnostic_recovery_pending_)
      {
        error = "connectAndValidate: pending stop reply ownership is unresolved";
        return false;
      }
    }
    if (!sendStop("startup/reconnect", error)) {
      error = "connectAndValidate: phase=connection category=transport_error "
        "operation=startup_reconnect_stop reason=" + visible_text(error);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (framing_state_ != SerialFramingState::synchronized ||
        diagnostic_recovery_pending_)
      {
        error = "connectAndValidate: startup stop reply ownership is unresolved";
        return false;
      }
    }
  } catch (const std::exception & ex) {
    error = "connectAndValidate: phase=connection category=transport_exception reason=" +
      visible_text(ex.what());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = SerialConnectionState::configuring;
    status_update_sequence_++;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker configuring and validating controller");
  }
  if (!validateControllerConfiguration(error)) {
    error = "connectAndValidate: configuration validation failed: " + error;
    return false;
  }
  if (!validateCommunication(error)) {
    error = "connectAndValidate: communication validation failed: " + error;
    return false;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker connection ready");
  }
  return true;
}

bool SerialIoWorker::sendOwnedCommands(
  const std::vector<std::string> & commands, std::string & error,
  std::chrono::steady_clock::time_point * write_accepted_at,
  bool * write_fully_accepted)
{
  uint64_t command_sequence = 0;
  uint64_t connection_generation = 0;
  if (config_.serial_command_log_observer) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    command_sequence = latest_submitted_sequence_;
    connection_generation = connection_generation_;
  }
  const auto result = transport_->commandTransaction(commands, config_.command_transaction_bounds);
  if (config_.serial_command_log_observer) {
    observeSerialCommandLog(
      SerialCommandLogEvent{
        result.write_accepted_at == std::chrono::steady_clock::time_point{} ?
        std::chrono::steady_clock::now() : result.write_accepted_at,
        command_sequence,
        connection_generation,
        commands});
  }
  last_command_transaction_owned_ = result.status == CommandTransportStatus::success;
  if (write_accepted_at != nullptr) {
    *write_accepted_at = result.write_accepted_at;
  }
  if (write_fully_accepted != nullptr) {
    *write_fully_accepted = result.write_fully_accepted;
  }
  if (result.status == CommandTransportStatus::success) {
    return true;
  }
  error = result.reason;
  if (result.status == CommandTransportStatus::failure) {
    return false;
  }

  // Ownership is unknown. Block all normal work and use the existing single bounded
  // drain/synchronization attempt before permitting another transaction.
  scheduleOwnershipRecovery(result.started_at);
  static const std::vector<std::string> exact_stop{
    "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
  if (commands != exact_stop) {
    (void)requestStop();
  }
  return result.write_fully_accepted;
}

void SerialIoWorker::scheduleOwnershipRecovery(
  std::chrono::steady_clock::time_point started_at)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  framing_state_ = SerialFramingState::unresolved;
  DiagnosticTransaction ownership_probe{"?FF\r", "FF=", config_.diagnostic_query_timeout};
  ownership_probe.correlation_id = ++next_validation_correlation_;
  ownership_probe.connection_generation = connection_generation_;
  ownership_probe.observer = [this](const DiagnosticPhaseEvent & event) {
      observeDiagnosticPhase(event);
    };
  timed_out_diagnostic_ = ownership_probe;
  timed_out_diagnostic_started_at_ = started_at;
  diagnostic_recovery_pending_ = true;
  last_recovery_correlation_ = ownership_probe.correlation_id;
  desired_command_.valid = false;
  minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
  status_update_sequence_++;
}

bool SerialIoWorker::sendStop(
  const char *, std::string & error,
  std::chrono::steady_clock::time_point * write_accepted_at,
  bool * write_fully_accepted)
{
  return sendOwnedCommands(
    {"!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"}, error, write_accepted_at,
    write_fully_accepted);
}

bool SerialIoWorker::executePendingRuntimeStop(std::string & error)
{
  uint64_t correlation = 0;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!runtime_stop_pending_) {
      return true;
    }
    correlation = runtime_stop_correlation_;
    generation = connection_generation_;
  }

  observeStopRequest(
    StopRequestEvent{
      StopRequestPhase::write_started, std::chrono::steady_clock::now(), correlation,
      generation, 0});
  std::chrono::steady_clock::time_point write_accepted_at;
  bool write_fully_accepted = false;
  const bool transaction_continues = sendStop(
    "continuing-runtime request", error, &write_accepted_at, &write_fully_accepted);
  observeStopRequest(
    StopRequestEvent{
      write_fully_accepted ? StopRequestPhase::write_accepted : StopRequestPhase::write_failed,
      write_fully_accepted && write_accepted_at != std::chrono::steady_clock::time_point{} ?
      write_accepted_at : std::chrono::steady_clock::now(), correlation, generation,
      write_fully_accepted ? std::size_t{28} : std::size_t{0}});
  if (!transaction_continues) {
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (runtime_stop_pending_ && runtime_stop_correlation_ == correlation) {
    runtime_stop_pending_ = false;
  }
  desired_command_.valid = false;
  minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
  applied_sequence_ = latest_submitted_sequence_;
  applied_stopped_ = true;
  status_update_sequence_++;
  return true;
}

void SerialIoWorker::observeStopRequest(const StopRequestEvent & event) const noexcept
{
  if (!config_.stop_request_observer) {
    return;
  }
  try {
    config_.stop_request_observer(event);
  } catch (...) {
    // Validation observability must never alter stop behavior.
  }
}

void SerialIoWorker::observeSerialCommandLog(const SerialCommandLogEvent & event) const noexcept
{
  if (!config_.serial_command_log_observer) {
    return;
  }
  try {
    config_.serial_command_log_observer(event);
  } catch (...) {
    // Command logging must never alter command transmission or recovery behavior.
  }
}

void SerialIoWorker::observeDiagnosticPhase(const DiagnosticPhaseEvent & event) const noexcept
{
  if (!config_.diagnostic_phase_observer) {
    return;
  }
  try {
    config_.diagnostic_phase_observer(event);
  } catch (...) {
    // Validation observability must never alter diagnostic behavior.
  }
}

void SerialIoWorker::observeTimeoutStop(const TimeoutStopEvent & event) const noexcept
{
  if (!config_.timeout_stop_observer) {
    return;
  }
  try {
    config_.timeout_stop_observer(event);
  } catch (...) {
    // Observability must never alter timeout-stop or recovery behavior.
  }
}

bool SerialIoWorker::sendDesiredCommand(const DesiredMotorCommand & command, std::string & error)
{
  return sendOwnedCommands(
    buildMotorCommands(command.channel_1_mps, command.channel_2_mps),
    error);
}

bool SerialIoWorker::validateControllerConfiguration(std::string & error)
{
  for (const auto & setting : config_.required_settings) {
    if (!executePendingRuntimeStop(error)) {
      error = "pending continuing-runtime stop failed: " + error;
      return false;
    }
    const std::string query = query_for_setting(setting);
    const std::string expected_prefix = setting.name + "=";
    const std::string expected_value = std::to_string(setting.expected_value);
    std::string response;
    try {
      if (!transport_->query(query, expected_prefix, response, error)) {
        error = validation_context(
          "configuration_validation", transport_failure_category(error), setting.name,
          setting.channel, query, expected_prefix, expected_value, response, error);
        return false;
      }
    } catch (const std::exception & ex) {
      error = validation_context(
        "configuration_validation", "transport_exception", setting.name, setting.channel, query,
        expected_prefix, expected_value, response,
        std::string("transport exception: ") + ex.what());
      return false;
    }
    const auto actual = protocol::parse_config_readback(response, setting.name);
    if (!actual.has_value()) {
      const bool has_expected_prefix = response.rfind(expected_prefix, 0) == 0;
      const std::string reason = has_expected_prefix ?
        malformed_numeric_reason(response, expected_prefix) : "wrong response prefix";
      error = validation_context(
        "configuration_validation", has_expected_prefix ? "malformed_response" : "wrong_prefix",
        setting.name, setting.channel, query, expected_prefix, expected_value, response, reason);
      return false;
    }
    if (*actual != setting.expected_value) {
      std::ostringstream stream;
      stream << "value mismatch; expected=" << setting.expected_value << " actual=" << *actual;
      error = validation_context(
        "configuration_validation", "value_mismatch", setting.name, setting.channel, query,
        expected_prefix, expected_value, response, stream.str());
      return false;
    }
  }
  return true;
}

bool SerialIoWorker::validateCommunication(std::string & error)
{
  if (!executePendingRuntimeStop(error)) {
    error = "pending continuing-runtime stop failed: " + error;
    return false;
  }
  const std::string query = "?FID\r";
  const std::string expected_prefix = "FID=";
  const std::string expected_value = "non-empty firmware identifier";
  std::string response;
  try {
    if (!transport_->query(query, expected_prefix, response, error)) {
      error = validation_context(
        "communication_validation", transport_failure_category(error), "FID", 0, query,
        expected_prefix, expected_value, response, error);
      return false;
    }
  } catch (const std::exception & ex) {
    error = validation_context(
      "communication_validation", "transport_exception", "FID", 0, query, expected_prefix,
      expected_value, response,
      std::string("transport exception: ") + ex.what());
    return false;
  }
  if (!protocol::parse_firmware_id(response).has_value()) {
    const bool has_expected_prefix = response.rfind(expected_prefix, 0) == 0;
    const std::string reason = has_expected_prefix ?
      "malformed firmware response: identifier is empty" : "wrong response prefix";
    error = validation_context(
      "communication_validation", has_expected_prefix ? "malformed_response" : "wrong_prefix",
      "FID", 0, query, expected_prefix, expected_value, response, reason);
    return false;
  }
  return true;
}

bool SerialIoWorker::pollEncoder(std::string & error)
{
  std::string response;
  const auto started_at = std::chrono::steady_clock::now();
  if (!transport_->query("?CR\r", "CR=", response, error)) {
    scheduleOwnershipRecovery(started_at);
    return true;
  }

  const auto parsed = protocol::parse_encoder_counts(response);
  if (!parsed.has_value()) {
    error = "malformed encoder response";
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  encoder_sequence_++;
  latest_encoder_sample_ = EncoderSample{
    parsed->first,
    parsed->second,
    std::chrono::steady_clock::now(),
    encoder_sequence_,
    true};
  status_update_sequence_++;
  return true;
}

bool SerialIoWorker::pollMotorTelemetry(std::string & error)
{
  (void)error;
  const auto & queries = motorTelemetryQueries();
  if (telemetry_query_index_ == 0) {
    telemetry_build_channel_1_ = MotorTelemetryChannel{1};
    telemetry_build_channel_2_ = MotorTelemetryChannel{2};
    telemetry_build_fault_flags_ = 0;
    telemetry_build_started_at_ = std::chrono::steady_clock::now();
    telemetry_failure_reason_.clear();
  }

  const auto & query = queries[telemetry_query_index_];
  std::string response;
  std::string query_error;
  const auto query_started = std::chrono::steady_clock::now();
  const bool query_ok = transport_->queryWithTimeout(
    query.command,
    query.expected_prefix,
    config_.telemetry_query_timeout,
    response,
    query_error);
  const auto query_elapsed = std::chrono::steady_clock::now() - query_started;
  if (!query_ok || query_elapsed > config_.telemetry_query_timeout) {
    telemetry_failure_reason_ = query_error.empty() ?
      "telemetry query failed or exceeded timeout" : query_error;
    scheduleOwnershipRecovery(query_started);
  } else {
    std::string parse_error;
    const auto value = parseMotorTelemetryInteger(response, query.expected_prefix, parse_error);
    if (!value.has_value()) {
      telemetry_failure_reason_ = parse_error;
    } else if (query.field == MotorTelemetryField::fault_flags) {
      telemetry_build_fault_flags_ = *value;
    } else {
      auto & channel = query.channel == 1 ? telemetry_build_channel_1_ : telemetry_build_channel_2_;
      switch (query.field) {
        case MotorTelemetryField::command_source: channel.command_source = *value; break;
        case MotorTelemetryField::applied_command: channel.applied_command = *value; break;
        case MotorTelemetryField::measured_speed: channel.measured_speed = *value; break;
        case MotorTelemetryField::current: channel.current = *value; break;
        case MotorTelemetryField::power: channel.power = *value; break;
        case MotorTelemetryField::motor_fault: channel.motor_fault = *value; break;
        case MotorTelemetryField::fault_flags: break;
      }
    }
  }

  if (!telemetry_failure_reason_.empty()) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_motor_telemetry_ = MotorTelemetrySnapshot{
      telemetry_build_channel_1_, telemetry_build_channel_2_, now, std::chrono::milliseconds(0),
      false, telemetry_failure_reason_, telemetry_sequence_, connection_generation_};
    latest_motor_telemetry_->channel_1.failure_reason = telemetry_failure_reason_;
    latest_motor_telemetry_->channel_2.failure_reason = telemetry_failure_reason_;
    telemetry_query_index_ = 0;
    return true;
  }

  ++telemetry_query_index_;
  if (telemetry_query_index_ < queries.size()) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  telemetry_build_channel_1_.timestamp = now;
  telemetry_build_channel_2_.timestamp = now;
  telemetry_build_channel_1_.valid = true;
  telemetry_build_channel_2_.valid = true;
  telemetry_build_channel_1_.fault_flags = telemetry_build_fault_flags_;
  telemetry_build_channel_2_.fault_flags = telemetry_build_fault_flags_;
  telemetry_build_channel_1_.failure_reason.clear();
  telemetry_build_channel_2_.failure_reason.clear();
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++telemetry_sequence_;
  latest_motor_telemetry_ = MotorTelemetrySnapshot{
    telemetry_build_channel_1_, telemetry_build_channel_2_, now, std::chrono::milliseconds(0),
    true, "", telemetry_sequence_, connection_generation_};
  telemetry_query_index_ = 0;
  return true;
}

bool SerialIoWorker::executeDiagnosticQuery(DiagnosticQueryKind query)
{
  auto transaction = diagnostic_transaction(query, config_.diagnostic_query_timeout);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    transaction.correlation_id = ++next_validation_correlation_;
    transaction.connection_generation = connection_generation_;
  }
  transaction.observer = [this](const DiagnosticPhaseEvent & event) {
      observeDiagnosticPhase(event);
    };
  observeDiagnosticPhase(
    DiagnosticPhaseEvent{
      DiagnosticPhase::selected, std::chrono::steady_clock::now(),
      transaction.correlation_id, transaction.connection_generation, transaction.command, 0});
  std::string priority_stop_error;
  if (!executePendingRuntimeStop(priority_stop_error)) {
    markFailure("stop before diagnostic selection failed: " + priority_stop_error);
    return false;
  }
  DiagnosticTransactionResult result;
  try {
    result = transport_->diagnosticQuery(transaction);
  } catch (const std::exception & ex) {
    result.status = DiagnosticTransportStatus::failure;
    result.started_at = std::chrono::steady_clock::now();
    result.reason = std::string("diagnostic transport exception: ") + ex.what();
  }
  const auto timestamp = std::chrono::steady_clock::now();
  SerialFramingState diagnostic_framing_state = SerialFramingState::synchronized;
  uint64_t diagnostic_generation = transaction.connection_generation;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_diagnostic_telemetry_ = DiagnosticTelemetry{
      query,
      result.raw_bytes.empty() ? result.response : result.raw_bytes,
      result.status == DiagnosticTransportStatus::success,
      timestamp,
      std::chrono::milliseconds(0),
      connection_generation_,
      transaction.correlation_id,
      result.started_at,
      result.write_accepted_at,
      result.first_byte_at,
      result.last_byte_at,
      result.timeout_at,
      result.delimiter_observed,
      result.status == DiagnosticTransportStatus::success ? "" : result.reason};
    status_update_sequence_++;
    if (result.status != DiagnosticTransportStatus::success) {
      framing_state_ = SerialFramingState::unresolved;
      timed_out_diagnostic_ = transaction;
      timed_out_diagnostic_started_at_ = result.started_at;
      diagnostic_recovery_pending_ = true;
      last_recovery_correlation_ = transaction.correlation_id;
      status_update_sequence_++;
    }
    diagnostic_framing_state = framing_state_;
    diagnostic_generation = connection_generation_;
  }
  if (config_.diagnostic_result_observer) {
    try {
      config_.diagnostic_result_observer(
        DiagnosticResultEvent{
          query,
          result.status,
          diagnostic_framing_state,
          result.response,
          result.raw_bytes,
          result.reason,
          result.delimiter_observed,
          result.started_at,
          result.write_accepted_at,
          result.first_byte_at,
          result.last_byte_at,
          result.timeout_at,
          result.completed_at,
          diagnostic_generation,
          transaction.correlation_id});
    } catch (...) {
      // Validation observability must never alter transport behavior.
    }
  }
  return true;
}

SerialIoWorker::DiagnosticRecoveryAttempt SerialIoWorker::performDiagnosticRecovery(
  std::string & error)
{
  DiagnosticTransaction timed_out;
  std::chrono::steady_clock::time_point started_at;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!diagnostic_recovery_pending_ || !timed_out_diagnostic_.has_value()) {
      diagnostic_recovery_start_reserved_ = false;
      error = "diagnostic recovery state is incomplete";
      return DiagnosticRecoveryAttempt::failed;
    }
    if (runtime_stop_pending_ || !diagnostic_recovery_start_reserved_) {
      diagnostic_recovery_start_reserved_ = false;
      return DiagnosticRecoveryAttempt::deferred;
    }
    diagnostic_recovery_start_reserved_ = false;
    timed_out = *timed_out_diagnostic_;
    started_at = timed_out_diagnostic_started_at_;
  }
  const auto sync = synchronization_transaction(
    timed_out, config_.diagnostic_recovery_bounds.synchronization_timeout);
  auto correlated_sync = sync;
  correlated_sync.correlation_id = timed_out.correlation_id;
  correlated_sync.connection_generation = timed_out.connection_generation;
  correlated_sync.observer = timed_out.observer;
  DiagnosticRecoveryResult recovered;
  try {
    recovered = transport_->boundedDiagnosticRecovery(
      timed_out, started_at, correlated_sync, config_.diagnostic_recovery_bounds,
      [this](std::string & checkpoint_error) {
        if (!executePendingRuntimeStop(checkpoint_error)) {
          if (checkpoint_error.empty()) {
            checkpoint_error = "pending stop failed before synchronization";
          }
          return false;
        }
        return true;
      });
  } catch (const std::exception & ex) {
    recovered.reason = std::string("diagnostic recovery transport exception: ") + ex.what();
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_diagnostic_telemetry_.has_value()) {
      latest_diagnostic_telemetry_->raw_value += recovered.drained_raw_bytes;
      if (recovered.drain_last_byte_at != std::chrono::steady_clock::time_point{}) {
        latest_diagnostic_telemetry_->last_byte_at = recovered.drain_last_byte_at;
      }
      if (recovered.synchronization_last_byte_at != std::chrono::steady_clock::time_point{}) {
        latest_diagnostic_telemetry_->last_byte_at = recovered.synchronization_last_byte_at;
      }
      latest_diagnostic_telemetry_->delimiter_observed =
        latest_diagnostic_telemetry_->delimiter_observed ||
        recovered.drain_delimiter_observed ||
        recovered.synchronization_delimiter_observed;
      if (!recovered.synchronized && !recovered.reason.empty()) {
        latest_diagnostic_telemetry_->failure_reason +=
          "; bounded recovery failed: " + recovered.reason;
      }
    }
    diagnostic_recovery_pending_ = false;
    diagnostic_recovery_start_reserved_ = false;
    timed_out_diagnostic_.reset();
    if (recovered.synchronized) {
      framing_state_ = SerialFramingState::synchronized;
      last_recovery_correlation_ = 0;
    }
    status_update_sequence_++;
  }
  if (!recovered.synchronized) {
    observeDiagnosticPhase(
      DiagnosticPhaseEvent{
        DiagnosticPhase::before_fallback_close, std::chrono::steady_clock::now(),
        timed_out.correlation_id, timed_out.connection_generation, timed_out.command,
        recovered.drained_raw_bytes.size()});
    error = "bounded diagnostic recovery failed: " + recovered.reason;
    return DiagnosticRecoveryAttempt::failed;
  }
  return DiagnosticRecoveryAttempt::completed;
}

void SerialIoWorker::invalidateDiagnosticTelemetry(const std::string & reason)
{
  if (!latest_diagnostic_telemetry_.has_value()) {
    return;
  }
  latest_diagnostic_telemetry_->valid = false;
  if (!latest_diagnostic_telemetry_->failure_reason.empty()) {
    latest_diagnostic_telemetry_->failure_reason += "; ";
  }
  latest_diagnostic_telemetry_->failure_reason += reason;
}

void SerialIoWorker::markFailure(const std::string & error)
{
  SerialConnectionState previous_state;
  bool issue_failure_stop = false;
  const char * previous_state_name = "unknown";
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_state = state_;
    issue_failure_stop =
      transport_open_ &&
      (state_ == SerialConnectionState::waiting_for_fresh_command ||
      state_ == SerialConnectionState::ready) &&
      framing_state_ == SerialFramingState::synchronized &&
      !diagnostic_recovery_pending_;
  }
  switch (previous_state) {
    case SerialConnectionState::disconnected:
      previous_state_name = "disconnected";
      break;
    case SerialConnectionState::connecting:
      previous_state_name = "connecting";
      break;
    case SerialConnectionState::configuring:
      previous_state_name = "configuring";
      break;
    case SerialConnectionState::waiting_for_fresh_command:
      previous_state_name = "waiting_for_fresh_command";
      break;
    case SerialConnectionState::ready:
      previous_state_name = "ready";
      break;
    case SerialConnectionState::unhealthy:
      previous_state_name = "unhealthy";
      break;
    case SerialConnectionState::reconnecting:
      previous_state_name = "reconnecting";
      break;
  }
  if (config_.log_callback) {
    std::ostringstream stream;
    stream << "Roboteq serial failure: " << error << "; connection_state_transition=" <<
      previous_state_name << "->unhealthy; reconnect scheduled";
    config_.log_callback(stream.str());
  }
  if (issue_failure_stop && transport_->isOpen()) {
    std::string ignored_error;
    sendStop("serial failure", ignored_error);
  }
  transport_->close();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = SerialConnectionState::unhealthy;
    transport_open_ = false;
    applied_stopped_ = true;
    desired_command_.valid = false;
    minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
    framing_state_ = SerialFramingState::unresolved;
    diagnostic_recovery_pending_ = false;
    diagnostic_recovery_start_reserved_ = false;
    timed_out_diagnostic_.reset();
    queued_diagnostic_query_.reset();
    invalidateDiagnosticTelemetry("connection failed; telemetry invalidated");
    status_update_sequence_++;
  }
}

std::vector<std::string> SerialIoWorker::buildMotorCommands(
  double channel_1_mps, double channel_2_mps) const
{
  std::ostringstream channel_1_cmd;
  std::ostringstream channel_2_cmd;

  if (config_.open_loop) {
    const auto powers = command_scaling::scale_pair_to_limit(
      channel_1_mps / config_.wheel_circumference * 60.0 / config_.max_rpm * 1000.0,
      channel_2_mps / config_.wheel_circumference * 60.0 / config_.max_rpm * 1000.0,
      1000.0);
    channel_1_cmd << "!G 1 " << static_cast<int32_t>(powers.first) << "\r";
    channel_2_cmd << "!G 2 " << static_cast<int32_t>(powers.second) << "\r";
  } else {
    const auto rpms = command_scaling::scale_pair_to_limit(
      channel_1_mps / config_.wheel_circumference * 60.0,
      channel_2_mps / config_.wheel_circumference * 60.0,
      config_.max_rpm);
    channel_1_cmd << "!S 1 " << static_cast<int32_t>(rpms.first) << "\r";
    channel_2_cmd << "!S 2 " << static_cast<int32_t>(rpms.second) << "\r";
  }

  return {channel_1_cmd.str(), channel_2_cmd.str()};
}

std::chrono::steady_clock::time_point SerialIoWorker::nextWakeTime(
  std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point next_encoder_poll,
  std::chrono::steady_clock::time_point next_reconnect) const
{
  auto next = std::min(next_encoder_poll, next_reconnect);
  if (desired_command_.valid && !applied_stopped_) {
    next = std::min(next, desired_command_.received_time + config_.command_timeout);
  }
  return std::max(now, next);
}

}  // namespace roboteq_ros2_driver
