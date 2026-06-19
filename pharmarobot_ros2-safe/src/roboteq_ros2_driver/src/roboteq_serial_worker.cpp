#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

#include "roboteq_ros2_driver/command_scaling.hpp"
#include "roboteq_ros2_driver/roboteq_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace roboteq_ros2_driver
{
namespace
{

std::string query_for_setting(const RequiredControllerSetting & setting)
{
  std::ostringstream query;
  query << "~" << setting.name;
  if (setting.channel > 0) {
    query << " " << setting.channel;
  }
  query << "\r";
  return query.str();
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
  latest_encoder_sample_.reset();
  return sample;
}

uint64_t SerialIoWorker::commandSequence() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_submitted_sequence_;
}

bool SerialIoWorker::isReady() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_ == ConnectionState::ready || state_ == ConnectionState::waiting_for_fresh_command;
}

void SerialIoWorker::run()
{
  auto next_encoder_poll = std::chrono::steady_clock::now();
  auto next_reconnect = std::chrono::steady_clock::now();

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
          ConnectionState::waiting_for_fresh_command : ConnectionState::ready;
      }
      next_encoder_poll = std::chrono::steady_clock::now() + config_.encoder_poll_period;
      next_reconnect = std::chrono::steady_clock::time_point::max();
    }

    DesiredMotorCommand command;
    bool have_command = false;
    bool timeout_stop_required = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      command = desired_command_;
      have_command = command.valid &&
        command.sequence > applied_sequence_ &&
        command.sequence >= minimum_motion_sequence_;

      if (desired_command_.valid && !applied_stopped_) {
        const auto age = std::chrono::steady_clock::now() - desired_command_.received_time;
        timeout_stop_required = age >= config_.command_timeout;
      }
    }

    std::string error;
    if (timeout_stop_required) {
      if (!sendStop("command timeout", error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied_stopped_ = true;
      applied_sequence_ = latest_submitted_sequence_;
      desired_command_.valid = false;
      minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
    } else if (have_command) {
      if (!sendDesiredCommand(command, error)) {
        markFailure(error);
        next_reconnect = std::chrono::steady_clock::now() + config_.reconnect_interval;
        continue;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied_sequence_ = command.sequence;
      applied_stopped_ =
        std::abs(command.channel_1_mps) < 1e-12 && std::abs(command.channel_2_mps) < 1e-12;
      state_ = ConnectionState::ready;
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

    std::unique_lock<std::mutex> lock(state_mutex_);
    state_cv_.wait_until(
      lock,
      nextWakeTime(std::chrono::steady_clock::now(), next_encoder_poll, next_reconnect),
      [this]() {
        return stop_requested_ ||
          (desired_command_.valid &&
          desired_command_.sequence > applied_sequence_ &&
          desired_command_.sequence >= minimum_motion_sequence_);
      });
  }

  std::string ignored_error;
  if (transport_->isOpen()) {
    sendStop("driver shutdown", ignored_error);
    transport_->close();
  }
}

bool SerialIoWorker::connectAndValidate(std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = ConnectionState::connecting;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker connecting");
  }
  if (!transport_->open(error)) {
    return false;
  }
  if (!sendStop("startup/reconnect", error)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = ConnectionState::configuring;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker configuring and validating controller");
  }
  if (!validateControllerConfiguration(error)) {
    return false;
  }
  if (!validateCommunication(error)) {
    return false;
  }
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker connection ready");
  }
  return true;
}

bool SerialIoWorker::sendStop(const char *, std::string & error)
{
  return transport_->sendCommands(
    {"!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"},
    error);
}

bool SerialIoWorker::sendDesiredCommand(const DesiredMotorCommand & command, std::string & error)
{
  return transport_->sendCommands(
    buildMotorCommands(command.channel_1_mps, command.channel_2_mps),
    error);
}

bool SerialIoWorker::validateControllerConfiguration(std::string & error)
{
  for (const auto & setting : config_.required_settings) {
    std::string response;
    if (!transport_->query(query_for_setting(setting), setting.name + "=", response, error)) {
      return false;
    }
    const auto actual = protocol::parse_config_readback(response, setting.name);
    if (!actual.has_value()) {
      error = "malformed readback for " + setting.name;
      return false;
    }
    if (*actual != setting.expected_value) {
      std::ostringstream stream;
      stream << "configuration mismatch for " << setting.name << ": expected "
             << setting.expected_value << " got " << *actual;
      error = stream.str();
      return false;
    }
  }
  return true;
}

bool SerialIoWorker::validateCommunication(std::string & error)
{
  std::string response;
  if (!transport_->query("?FID\r", "FID=", response, error)) {
    return false;
  }
  if (!protocol::parse_firmware_id(response).has_value()) {
    error = "malformed firmware response";
    return false;
  }
  return true;
}

bool SerialIoWorker::pollEncoder(std::string & error)
{
  std::string response;
  if (!transport_->query("?CR\r", "CR=", response, error)) {
    return false;
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
  return true;
}

void SerialIoWorker::markFailure(const std::string &)
{
  if (config_.log_callback) {
    config_.log_callback("Roboteq serial worker entering reconnect after serial failure");
  }
  if (transport_->isOpen()) {
    std::string ignored_error;
    sendStop("serial failure", ignored_error);
  }
  transport_->close();
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = ConnectionState::unhealthy;
  applied_stopped_ = true;
  desired_command_.valid = false;
  minimum_motion_sequence_ = latest_submitted_sequence_ + 1;
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
