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

bool SerialIoWorker::isReady() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_ == SerialConnectionState::ready ||
    state_ == SerialConnectionState::waiting_for_fresh_command;
}

SerialWorkerStatus SerialIoWorker::status() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return SerialWorkerStatus{
    state_,
    transport_->isOpen(),
    state_ == SerialConnectionState::ready ||
      state_ == SerialConnectionState::waiting_for_fresh_command,
    latest_encoder_sample_.has_value() || last_encoder_sample_.has_value(),
    config_.require_fresh_command_after_reconnect,
    latest_encoder_sample_ ? latest_encoder_sample_->timestamp :
      (last_encoder_sample_ ? last_encoder_sample_->timestamp :
      std::chrono::steady_clock::time_point{}),
    latest_encoder_sample_ ? latest_encoder_sample_->sequence :
      (last_encoder_sample_ ? last_encoder_sample_->sequence : 0),
    latest_submitted_sequence_,
  };
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
          SerialConnectionState::waiting_for_fresh_command : SerialConnectionState::ready;
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
      state_ = SerialConnectionState::ready;
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
    state_ = SerialConnectionState::connecting;
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
    if (!sendStop("startup/reconnect", error)) {
      error = "connectAndValidate: phase=connection category=transport_error "
        "operation=startup_reconnect_stop reason=" + visible_text(error);
      return false;
    }
  } catch (const std::exception & ex) {
    error = "connectAndValidate: phase=connection category=transport_exception reason=" +
      visible_text(ex.what());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = SerialConnectionState::configuring;
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

void SerialIoWorker::markFailure(const std::string & error)
{
  SerialConnectionState previous_state;
  const char * previous_state_name = "unknown";
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_state = state_;
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
  if (transport_->isOpen()) {
    std::string ignored_error;
    sendStop("serial failure", ignored_error);
  }
  transport_->close();
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = SerialConnectionState::unhealthy;
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
