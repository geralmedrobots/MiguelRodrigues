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
//    * Neither the name of the Geralmedrobots nor the names of its
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

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "roboteq_ros2_driver/driver_parameter_validation.hpp"
#include "roboteq_ros2_driver/roboteq_diagnostics.hpp"

namespace driver = roboteq_ros2_driver;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

driver::DiagnosticsState baseState()
{
  driver::DiagnosticsState state;
  state.serial_connected = true;
  state.serial_ready = true;
  state.command_active = true;
  state.command_timed_out = false;
  state.command_age = std::chrono::milliseconds(100);
  state.encoder_sample_available = true;
  state.encoder_age = std::chrono::milliseconds(100);
  state.controller_faults = driver::ControllerSafetySignal::normal;
  state.sto_status = driver::ControllerSafetySignal::normal;
  return state;
}

driver::DiagnosticsConfig baseConfig()
{
  driver::DiagnosticsConfig config;
  config.publish_period = std::chrono::milliseconds(1000);
  config.encoder_freshness_warn = std::chrono::milliseconds(250);
  config.encoder_freshness_error = std::chrono::milliseconds(1000);
  config.command_watchdog_warn = std::chrono::milliseconds(250);
  config.command_watchdog_error = std::chrono::milliseconds(1000);
  return config;
}

roboteq_ros2_driver::SerialWorkerStatus workerStatus(
  roboteq_ros2_driver::SerialConnectionState connection_state)
{
  roboteq_ros2_driver::SerialWorkerStatus status;
  status.connection_state = connection_state;
  status.transport_open = connection_state !=
    roboteq_ros2_driver::SerialConnectionState::disconnected;
  status.ready_for_motion = connection_state ==
    roboteq_ros2_driver::SerialConnectionState::ready;
  return status;
}

const DiagnosticStatus & statusByName(
  const diagnostic_msgs::msg::DiagnosticArray & msg,
  const std::string & name)
{
  for (const auto & status : msg.status) {
    if (status.name == name) {
      return status;
    }
  }
  throw std::runtime_error("missing diagnostic status");
}

std::string valueByKey(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  throw std::runtime_error("missing diagnostic value");
}

driver::DiagnosticsPublicationDecision evaluate(
  driver::DiagnosticsPublisherState & publisher_state,
  const driver::DiagnosticsState & state,
  const driver::DiagnosticsConfig & config,
  std::chrono::steady_clock::time_point now,
  const rclcpp::Time & stamp = rclcpp::Time(1, 0, RCL_ROS_TIME))
{
  const auto msg = driver::buildDiagnosticsArray(stamp, state, config);
  return publisher_state.evaluate(msg, now, config.publish_period);
}

}  // namespace

TEST(RoboteqDiagnostics, SeverityTracksStaleThresholds)
{
  auto state = baseState();
  auto config = baseConfig();

  state.encoder_age = std::chrono::milliseconds(300);
  auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").level, DiagnosticStatus::WARN);

  state.encoder_age = std::chrono::milliseconds(1200);
  msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").level, DiagnosticStatus::ERROR);
}

TEST(DriverParameterValidation, RejectsInvalidEncoderFreshnessThresholds)
{
  namespace validation = roboteq_ros2_driver::parameter_validation;
  for (const double value : {
    0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()
  })
  {
    auto error = validation::validate_encoder_freshness_thresholds(value, 1.0);
    ASSERT_TRUE(error.has_value());
    EXPECT_EQ(error->parameter, "encoder_freshness_warn_s");

    error = validation::validate_encoder_freshness_thresholds(0.25, value);
    ASSERT_TRUE(error.has_value());
    EXPECT_EQ(error->parameter, "encoder_freshness_error_s");
  }
}

TEST(DriverParameterValidation, RequiresEncoderFreshnessErrorGreaterThanWarn)
{
  namespace validation = roboteq_ros2_driver::parameter_validation;
  for (const double error_threshold : {0.25, 0.1}) {
    const auto error = validation::validate_encoder_freshness_thresholds(
      0.25, error_threshold);
    ASSERT_TRUE(error.has_value());
    EXPECT_EQ(error->parameter, "encoder_freshness_error_s");
  }

  EXPECT_FALSE(
    validation::validate_encoder_freshness_thresholds(0.25, 1.0).has_value());
}

TEST(RoboteqDiagnostics, PublishesFirstMessage)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();

  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    std::chrono::steady_clock::time_point{});

  EXPECT_TRUE(decision.publish);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_FALSE(decision.periodic);
}

TEST(RoboteqDiagnostics, SuppressesUnchangedStateBeforePeriodEvenWhenStampAndAgeChange)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(evaluate(publisher_state, state, config, start).publish);

  state.command_age = std::chrono::milliseconds(200);
  state.encoder_age = std::chrono::milliseconds(200);
  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    start + std::chrono::milliseconds(500),
    rclcpp::Time(2, 0, RCL_ROS_TIME));

  EXPECT_FALSE(decision.publish);
  EXPECT_FALSE(decision.state_changed);
  EXPECT_FALSE(decision.periodic);
}

TEST(RoboteqDiagnostics, PublishesUnchangedStateAfterPeriod)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(evaluate(publisher_state, state, config, start).publish);
  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    start + config.publish_period);

  EXPECT_TRUE(decision.publish);
  EXPECT_FALSE(decision.state_changed);
  EXPECT_TRUE(decision.periodic);
}

TEST(RoboteqDiagnostics, PublishesImmediatelyOnStateChangeBeforePeriod)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_TRUE(evaluate(publisher_state, state, config, start).publish);
  state.encoder_age = std::chrono::milliseconds(300);
  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    start + std::chrono::milliseconds(10));

  EXPECT_TRUE(decision.publish);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_FALSE(decision.periodic);
}

TEST(RoboteqDiagnostics, RecoveryChangesFingerprintAndPublishesAgain)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();
  const auto start = std::chrono::steady_clock::time_point{};

  state.serial_connected = false;
  state.serial_ready = false;
  EXPECT_TRUE(evaluate(publisher_state, state, config, start).publish);

  state.serial_connected = true;
  state.serial_ready = true;
  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    start + std::chrono::milliseconds(10));
  state.serial_connected = true;
  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);

  EXPECT_TRUE(decision.publish);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_EQ(statusByName(msg, "roboteq/serial_connection").level, DiagnosticStatus::OK);
}

TEST(RoboteqDiagnostics, SerialDisconnectAndReconnectUseFailureAndRecoverySeverity)
{
  auto state = baseState();
  auto config = baseConfig();

  state.serial_connected = false;
  state.serial_ready = false;
  auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/serial_connection").level, DiagnosticStatus::ERROR);

  state.serial_connected = true;
  state.serial_ready = false;
  msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/serial_connection").level, DiagnosticStatus::WARN);

  state.serial_ready = true;
  msg = driver::buildDiagnosticsArray(rclcpp::Time(3, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/serial_connection").level, DiagnosticStatus::OK);
  EXPECT_EQ(statusByName(msg, "roboteq/serial_connection").message, "ready");
}

TEST(RoboteqDiagnostics, WaitingForFreshCommandAfterReconnectIsDegraded)
{
  auto state = baseState();
  auto config = baseConfig();
  state.serial_connected = true;
  state.serial_ready = true;
  state.worker_status = workerStatus(
    roboteq_ros2_driver::SerialConnectionState::waiting_for_fresh_command);

  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  const auto & serial_status = statusByName(msg, "roboteq/serial_connection");

  EXPECT_EQ(serial_status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(serial_status.message, "waiting for fresh command");
  EXPECT_EQ(valueByKey(serial_status, "connection_state"), "waiting_for_fresh_command");
}

TEST(RoboteqDiagnostics, UnresolvedDiagnosticFramingNeverReportsReady)
{
  auto state = baseState();
  auto config = baseConfig();
  state.worker_status = workerStatus(
    roboteq_ros2_driver::SerialConnectionState::ready);
  state.worker_status->connection_generation = 7;
  state.worker_status->framing_state = roboteq_ros2_driver::SerialFramingState::unresolved;
  state.worker_status->diagnostic_recovery_pending = true;

  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  const auto & serial_status = statusByName(msg, "roboteq/serial_connection");

  EXPECT_EQ(serial_status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(serial_status.message, "diagnostic framing unresolved");
  EXPECT_EQ(valueByKey(serial_status, "connection_generation"), "7");
  EXPECT_EQ(valueByKey(serial_status, "serial_framing"), "unresolved");
  EXPECT_EQ(valueByKey(serial_status, "diagnostic_recovery_pending"), "true");
}

TEST(RoboteqDiagnostics, CommandTimeoutAndRecoveryUseErrorAndInfoSeverity)
{
  auto state = baseState();
  auto config = baseConfig();

  state.command_timed_out = true;
  state.command_age = std::chrono::milliseconds(1200);
  auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/command_watchdog").level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statusByName(msg, "roboteq/command_watchdog").message, "command timeout");

  state.command_timed_out = false;
  state.command_age = std::chrono::milliseconds(10);
  msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/command_watchdog").level, DiagnosticStatus::OK);
  EXPECT_EQ(statusByName(msg, "roboteq/command_watchdog").message, "command fresh");
}

TEST(RoboteqDiagnostics, CommandAgeWarnsBeforeTimeout)
{
  auto state = baseState();
  auto config = baseConfig();
  state.command_timed_out = false;
  state.command_age = std::chrono::milliseconds(300);

  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  const auto & watchdog = statusByName(msg, "roboteq/command_watchdog");

  EXPECT_EQ(watchdog.level, DiagnosticStatus::WARN);
  EXPECT_EQ(watchdog.message, "command stale");
}

TEST(RoboteqDiagnostics, EncoderStaleAndFreshRecoveryUseConfiguredThresholds)
{
  auto state = baseState();
  auto config = baseConfig();

  state.encoder_age = std::chrono::milliseconds(1000);
  auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").message, "stale");

  state.encoder_age = std::chrono::milliseconds(10);
  msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").level, DiagnosticStatus::OK);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").message, "fresh");
}

TEST(RoboteqDiagnostics, EncoderFreshRecoveryPublishesImmediately)
{
  driver::DiagnosticsPublisherState publisher_state;
  auto state = baseState();
  auto config = baseConfig();
  const auto start = std::chrono::steady_clock::time_point{};

  state.encoder_age = std::chrono::milliseconds(1200);
  EXPECT_TRUE(evaluate(publisher_state, state, config, start).publish);

  state.encoder_age = std::chrono::milliseconds(10);
  const auto decision = evaluate(
    publisher_state,
    state,
    config,
    start + std::chrono::milliseconds(10));

  EXPECT_TRUE(decision.publish);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_FALSE(decision.periodic);
}

TEST(RoboteqDiagnostics, MissingEncoderSampleNeverReportsHealthy)
{
  auto state = baseState();
  auto config = baseConfig();

  state.encoder_sample_available = false;
  state.encoder_age = std::chrono::milliseconds(10);
  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);

  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").level, DiagnosticStatus::WARN);
  EXPECT_EQ(statusByName(msg, "roboteq/encoder_freshness").message, "no samples yet");
}

TEST(RoboteqDiagnostics, UnsupportedUnknownAndActiveFaultStoAreNotHealthy)
{
  auto state = baseState();
  auto config = baseConfig();

  state.controller_faults = driver::ControllerSafetySignal::unsupported;
  state.sto_status = driver::ControllerSafetySignal::unknown;
  auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/controller_faults").level, DiagnosticStatus::WARN);
  EXPECT_EQ(statusByName(msg, "roboteq/controller_faults").message, "unsupported");
  EXPECT_EQ(statusByName(msg, "roboteq/controller_sto").level, DiagnosticStatus::WARN);
  EXPECT_EQ(statusByName(msg, "roboteq/controller_sto").message, "unknown");

  state.controller_faults = driver::ControllerSafetySignal::active;
  state.sto_status = driver::ControllerSafetySignal::active;
  msg = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);
  EXPECT_EQ(statusByName(msg, "roboteq/controller_faults").level, DiagnosticStatus::ERROR);
  EXPECT_EQ(statusByName(msg, "roboteq/controller_sto").level, DiagnosticStatus::ERROR);
}

TEST(RoboteqDiagnostics, FingerprintIgnoresHeaderStampAndAgeValue)
{
  auto state = baseState();
  auto config = baseConfig();
  auto first = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);

  state.encoder_age = std::chrono::milliseconds(200);
  state.command_age = std::chrono::milliseconds(200);
  auto second = driver::buildDiagnosticsArray(rclcpp::Time(2, 0, RCL_ROS_TIME), state, config);

  EXPECT_EQ(driver::diagnosticsFingerprint(first), driver::diagnosticsFingerprint(second));
  EXPECT_NE(
    valueByKey(statusByName(first, "roboteq/encoder_freshness"), "age"),
    valueByKey(statusByName(second, "roboteq/encoder_freshness"), "age"));
}

TEST(RoboteqDiagnostics, BuildsLogRecordsWithMatchingSeverityAndMessages)
{
  auto state = baseState();
  auto config = baseConfig();
  state.serial_connected = false;
  state.serial_ready = false;
  state.command_timed_out = true;
  state.encoder_age = std::chrono::milliseconds(300);
  state.controller_faults = driver::ControllerSafetySignal::normal;
  state.sto_status = driver::ControllerSafetySignal::unknown;

  const auto msg = driver::buildDiagnosticsArray(rclcpp::Time(1, 0, RCL_ROS_TIME), state, config);
  const auto logs = driver::buildDiagnosticsLogRecords(msg);

  ASSERT_EQ(logs.size(), msg.status.size());
  EXPECT_EQ(logs[0].level, DiagnosticStatus::ERROR);
  EXPECT_NE(logs[0].message.find("roboteq/serial_connection: disconnected"), std::string::npos);
  EXPECT_EQ(logs[2].level, DiagnosticStatus::WARN);
  EXPECT_NE(logs[2].message.find("roboteq/encoder_freshness: stale"), std::string::npos);
  EXPECT_EQ(logs[3].level, DiagnosticStatus::OK);
  EXPECT_EQ(logs[4].level, DiagnosticStatus::WARN);
}
