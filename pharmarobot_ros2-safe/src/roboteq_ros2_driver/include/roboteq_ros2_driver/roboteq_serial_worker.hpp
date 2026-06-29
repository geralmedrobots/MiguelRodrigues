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
};

struct SerialWorkerConfig
{
  bool open_loop{false};
  double wheel_circumference{1.0};
  int max_rpm{100};
  std::chrono::milliseconds command_timeout{500};
  std::chrono::milliseconds encoder_poll_period{50};
  std::chrono::milliseconds reconnect_interval{1000};
  bool require_fresh_command_after_reconnect{true};
  std::vector<configuration::RequiredControllerSetting> required_settings;
  std::function<void(const std::string &)> log_callback;
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
  void submitCommand(double channel_1_mps, double channel_2_mps);
  void invalidateCommands();
  std::optional<EncoderSample> takeLatestEncoderSample();
  uint64_t commandSequence() const;
  bool isConnected() const;
  bool isReadyForMotion() const;
  SerialWorkerStatus status() const;

private:
  void run();
  bool connectAndValidate(std::string & error);
  bool sendStop(const char * reason, std::string & error);
  bool sendDesiredCommand(const DesiredMotorCommand & command, std::string & error);
  bool validateControllerConfiguration(std::string & error);
  bool validateCommunication(std::string & error);
  bool pollEncoder(std::string & error);
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
  bool stop_requested_{false};
  bool worker_started_{false};
  std::optional<EncoderSample> latest_encoder_sample_;
  std::optional<EncoderSample> last_encoder_sample_;
  uint64_t encoder_sequence_{0};
  uint64_t status_update_sequence_{0};
  SerialConnectionState state_{SerialConnectionState::disconnected};
  std::thread worker_thread_;
};

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_WORKER_HPP_
