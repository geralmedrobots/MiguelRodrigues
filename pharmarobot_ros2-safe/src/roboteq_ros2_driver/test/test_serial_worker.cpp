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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>

#include <algorithm>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "roboteq_ros2_driver/phase5b_harness_logic.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

struct FakeTransportState
{
  struct WriteObservation
  {
    std::vector<std::string> commands;
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::time_point completed;
    bool succeeded;
  };

  mutable std::mutex mutex;
  std::condition_variable cv;
  std::atomic<bool> open{false};
  std::atomic<int> forbidden_is_open_calls{0};
  std::thread::id forbidden_is_open_thread{};
  bool fail_next_write{false};
  std::string encoder_response{"CR=10:20"};
  int open_calls{0};
  int fail_open_after{-1};
  int close_calls{0};
  std::chrono::milliseconds write_delay{0};
  std::chrono::milliseconds query_delay{0};
  std::vector<std::vector<std::string>> write_batches;
  std::vector<WriteObservation> write_observations;
  std::vector<std::string> queries;
  std::vector<driver::StartupDrainResult> startup_drains;
  std::vector<std::string> logs;
  std::vector<std::thread::id> transport_thread_ids;
  std::vector<std::string> events;
  std::string failing_query;
  std::string injected_response;
  std::string injected_error;
  driver::StartupDrainResult startup_drain_result{true, "", ""};
  bool fail_query{false};
  bool throw_query_exception{false};
  driver::DiagnosticTransactionResult diagnostic_result{
    driver::DiagnosticTransportStatus::success, "FF=0", "FF=0\r", "", {}};
  driver::DiagnosticRecoveryResult recovery_result{true, "", "FS=0", ""};
  std::vector<driver::DiagnosticTransaction> diagnostic_transactions;
  std::vector<driver::DiagnosticTransaction> synchronization_transactions;
  int recovery_calls{0};
  int recovery_completed{0};
  bool hold_diagnostic_result{false};
  bool diagnostic_entered{false};
  bool release_diagnostic_result{false};
  bool hold_recovery{false};
  bool recovery_entered{false};
  bool release_recovery{false};
  bool hold_encoder_query{false};
  bool encoder_query_entered{false};
  bool release_encoder_query{false};
  bool hold_write{false};
  bool write_entered{false};
  bool release_write{false};
  bool hold_command_ack_collection{false};
  bool command_ack_collection_entered{false};
  bool release_command_ack_collection{false};
  std::vector<driver::CommandTransportStatus> command_statuses;
  std::vector<bool> command_full_acceptance;
  std::size_t command_status_index{0};
};

struct TimeoutEventCollector
{
  void observe(const driver::TimeoutStopEvent & event)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      events.push_back(event);
    }
    cv.notify_all();
  }

  bool waitForCompletion(std::chrono::milliseconds timeout = 500ms)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(
      lock, timeout, [this]() {
        return std::any_of(
          events.begin(), events.end(), [](const auto & event) {
            return event.phase == driver::TimeoutStopEventPhase::zero_write_completed;
          });
      });
  }

  std::vector<driver::TimeoutStopEvent> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return events;
  }

  mutable std::mutex mutex;
  std::condition_variable cv;
  std::vector<driver::TimeoutStopEvent> events;
};

struct ValidationEventCollector
{
  void observeStop(const driver::StopRequestEvent & event)
  {
    std::lock_guard<std::mutex> lock(mutex);
    stop_events.push_back(event);
    cv.notify_all();
  }

  void observeDiagnostic(const driver::DiagnosticPhaseEvent & event)
  {
    std::lock_guard<std::mutex> lock(mutex);
    diagnostic_events.push_back(event);
    cv.notify_all();
  }

  bool waitForStopAccepted(std::chrono::milliseconds timeout = 500ms)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(
      lock, timeout, [this]() {
        return std::any_of(
          stop_events.begin(), stop_events.end(), [](const auto & event) {
            return event.phase == driver::StopRequestPhase::write_accepted;
          });
      });
  }

  mutable std::mutex mutex;
  std::condition_variable cv;
  std::vector<driver::StopRequestEvent> stop_events;
  std::vector<driver::DiagnosticPhaseEvent> diagnostic_events;
};

class FakeTransport : public driver::IRoboteqSerialTransport
{
public:
  explicit FakeTransport(std::shared_ptr<FakeTransportState> state)
  : state_(std::move(state))
  {
  }

  bool open(std::string & error) override
  {
    recordThread();
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->open_calls++;
    if (state_->fail_open_after >= 0 && state_->open_calls > state_->fail_open_after) {
      state_->open = false;
      error = "injected open failure";
      return false;
    }
    state_->open = true;
    return true;
  }

  void close() noexcept override
  {
    recordThread();
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->open = false;
    state_->close_calls++;
  }

  bool isOpen() const noexcept override
  {
    if (std::this_thread::get_id() == state_->forbidden_is_open_thread) {
      state_->forbidden_is_open_calls.fetch_add(1);
    }
    return state_->open.load();
  }

  driver::StartupDrainResult drainStartupInput(
    const driver::StartupDrainBounds &) override
  {
    recordThread();
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto result = state_->startup_drain_result;
    result.started_at = std::chrono::steady_clock::now();
    result.completed_at = result.started_at;
    state_->startup_drains.push_back(result);
    state_->cv.notify_all();
    return result;
  }

  bool sendCommands(const std::vector<std::string> & commands, std::string & error)
  {
    recordThread();
    const auto started = std::chrono::steady_clock::now();
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      if (state_->hold_write) {
        state_->write_entered = true;
        state_->cv.notify_all();
        state_->cv.wait(lock, [this]() {return state_->release_write;});
      }
    }
    if (state_->write_delay.count() > 0) {
      std::this_thread::sleep_for(state_->write_delay);
    }

    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->fail_next_write) {
      state_->fail_next_write = false;
      error = "injected write failure";
      state_->write_observations.push_back(
        {commands, started, std::chrono::steady_clock::now(), false});
      return false;
    }
    state_->events.push_back(commands.empty() ? "write:<empty>" : "write:" + commands.front());
    state_->write_batches.push_back(commands);
    state_->write_observations.push_back(
      {commands, started, std::chrono::steady_clock::now(), true});
    return true;
  }

  driver::CommandTransactionResult commandTransaction(
    const std::vector<std::string> & commands,
    const driver::CommandTransactionBounds &) override
  {
    driver::CommandTransactionResult result;
    result.started_at = std::chrono::steady_clock::now();
    std::string error;
    const bool ok = sendCommands(commands, error);
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      if (state_->hold_command_ack_collection) {
        state_->command_ack_collection_entered = true;
        state_->cv.notify_all();
        state_->cv.wait(lock, [this]() {return state_->release_command_ack_collection;});
      }
    }
    result.write_accepted_at = std::chrono::steady_clock::now();
    result.write_fully_accepted = ok;
    result.completed_at = result.write_accepted_at;
    result.expected_acknowledgements = commands.size();
    result.received_acknowledgements = ok ? commands.size() : 0;
    result.status = ok ? driver::CommandTransportStatus::success :
      driver::CommandTransportStatus::failure;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->command_status_index < state_->command_statuses.size()) {
        const auto index = state_->command_status_index++;
        result.status = state_->command_statuses[index];
        if (index < state_->command_full_acceptance.size()) {
          result.write_fully_accepted = state_->command_full_acceptance[index];
          if (!result.write_fully_accepted) {
            result.write_accepted_at = {};
          }
        }
      }
    }
    if (result.status == driver::CommandTransportStatus::unresolved) {
      result.reason = "injected unresolved command acknowledgement ownership";
      result.received_acknowledgements = commands.empty() ? 0 : commands.size() - 1;
    }
    if (!error.empty() || result.reason.empty()) {
      result.reason = error;
    }
    return result;
  }

  bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) override
  {
    recordThread();
    if (state_->query_delay.count() > 0) {
      std::this_thread::sleep_for(state_->query_delay);
    }
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->queries.push_back(command);
    if (command == state_->failing_query) {
      response = state_->injected_response;
      if (state_->throw_query_exception) {
        throw std::runtime_error(state_->injected_error);
      }
      if (state_->fail_query) {
        error = state_->injected_error;
        return false;
      }
      return true;
    }
    if (expected_prefix == "FID=") {
      response = "FID=fake";
      return true;
    }
    if (expected_prefix == "CR=") {
      state_->events.push_back("encoder");
      state_->encoder_query_entered = true;
      state_->cv.notify_all();
      if (state_->hold_encoder_query) {
        state_->cv.wait(lock, [this]() {return state_->release_encoder_query;});
      }
      response = state_->encoder_response;
      return true;
    }
    response = expected_prefix + "1";
    return true;
  }

  driver::DiagnosticTransactionResult diagnosticQuery(
    const driver::DiagnosticTransaction & transaction) override
  {
    recordThread();
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->diagnostic_transactions.push_back(transaction);
    state_->events.push_back("diagnostic:" + transaction.command);
    state_->diagnostic_entered = true;
    state_->cv.notify_all();
    if (state_->hold_diagnostic_result) {
      state_->cv.wait(lock, [this]() {return state_->release_diagnostic_result;});
    }
    auto result = state_->diagnostic_result;
    result.started_at = std::chrono::steady_clock::now();
    return result;
  }

  driver::DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const driver::DiagnosticTransaction &,
    std::chrono::steady_clock::time_point,
    const driver::DiagnosticTransaction & synchronization_transaction,
    const driver::DiagnosticRecoveryBounds &,
    const std::function<bool(std::string &)> & before_synchronization) override
  {
    recordThread();
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->recovery_calls++;
    state_->events.push_back("recovery");
    state_->synchronization_transactions.push_back(synchronization_transaction);
    state_->recovery_entered = true;
    state_->cv.notify_all();
    if (state_->hold_recovery) {
      state_->cv.wait(lock, [this]() {return state_->release_recovery;});
    }
    lock.unlock();
    std::string checkpoint_error;
    if (before_synchronization && !before_synchronization(checkpoint_error)) {
      return driver::DiagnosticRecoveryResult{false, "", "", checkpoint_error};
    }
    lock.lock();
    state_->events.push_back("synchronization");
    state_->recovery_completed++;
    state_->cv.notify_all();
    return state_->recovery_result;
  }

private:
  void recordThread() const
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->transport_thread_ids.push_back(std::this_thread::get_id());
  }

  std::shared_ptr<FakeTransportState> state_;
};

driver::SerialWorkerConfig workerConfig()
{
  driver::SerialWorkerConfig config;
  config.open_loop = false;
  config.wheel_circumference = 1.0;
  config.max_rpm = 100;
  config.command_timeout = 40ms;
  config.encoder_poll_period = 30ms;
  config.reconnect_interval = 40ms;
  config.require_fresh_command_after_reconnect = true;
  return config;
}

std::vector<std::vector<std::string>> writeBatches(
  const std::shared_ptr<FakeTransportState> & state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  return state->write_batches;
}

std::vector<FakeTransportState::WriteObservation> writeObservations(
  const std::shared_ptr<FakeTransportState> & state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  return state->write_observations;
}

bool waitForWriteBatches(
  const std::shared_ptr<FakeTransportState> & state,
  std::size_t count,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (writeBatches(state).size() >= count) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

bool waitForCommand(
  const std::shared_ptr<FakeTransportState> & state,
  const std::string & command,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto batches = writeBatches(state);
    if (std::any_of(
        batches.begin(),
        batches.end(),
        [&command](const auto & batch) {
          return std::find(batch.begin(), batch.end(), command) != batch.end();
        }))
    {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

bool isStopBatch(const std::vector<std::string> & batch)
{
  return batch == std::vector<std::string>{"!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
}

void expectBoundedFakeWriteLatency(
  const FakeTransportState::WriteObservation & observation,
  std::chrono::milliseconds injected_delay)
{
  const auto elapsed = observation.completed - observation.started;
  EXPECT_GE(elapsed, injected_delay);
  EXPECT_LT(elapsed, 250ms);
}

bool hasEncoderQuery(const std::shared_ptr<FakeTransportState> & state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  return std::find(
    state->queries.begin(),
    state->queries.end(),
    "?CR\r") != state->queries.end();
}

bool waitForRecoveryCalls(
  const std::shared_ptr<FakeTransportState> & state,
  int count,
  std::chrono::milliseconds timeout = 500ms)
{
  std::unique_lock<std::mutex> lock(state->mutex);
  return state->cv.wait_for(lock, timeout, [&]() {return state->recovery_completed >= count;});
}

driver::SerialWorkerStatus waitForWorkerState(
  const driver::SerialIoWorker & worker,
  driver::SerialConnectionState expected_state,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  driver::SerialWorkerStatus status;
  while (std::chrono::steady_clock::now() < deadline) {
    status = worker.status();
    if (status.connection_state == expected_state) {
      return status;
    }
    std::this_thread::sleep_for(5ms);
  }
  return status;
}

driver::SerialWorkerStatus waitForFramingState(
  const driver::SerialIoWorker & worker,
  driver::SerialFramingState expected_state,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  driver::SerialWorkerStatus status;
  while (std::chrono::steady_clock::now() < deadline) {
    status = worker.status();
    if (status.framing_state == expected_state) {
      return status;
    }
    std::this_thread::sleep_for(5ms);
  }
  return status;
}

void expectReadinessConsistent(const driver::SerialIoWorker & worker)
{
  const auto status = worker.status();
  EXPECT_EQ(worker.isConnected(), status.transport_open);
  EXPECT_EQ(worker.isReadyForMotion(), status.ready_for_motion);
}

std::string runConfigurationFailure(
  const std::shared_ptr<FakeTransportState> & state,
  int expected_value = 1,
  bool include_required_setting = true)
{
  auto config = workerConfig();
  config.reconnect_interval = 1000ms;
  config.encoder_poll_period = 1000ms;
  if (include_required_setting) {
    config.required_settings.push_back({"KP", 1, expected_value});
  }
  config.log_callback = [state](const std::string & message) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->logs.push_back(message);
    };

  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();

  std::string failure_log;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      const auto failure = std::find_if(
        state->logs.begin(), state->logs.end(), [](const std::string & message) {
          return message.find("Roboteq serial failure:") != std::string::npos;
        });
      if (failure != state->logs.end()) {
        failure_log = *failure;
        break;
      }
    }
    std::this_thread::sleep_for(5ms);
  }
  worker.stop();
  return failure_log;
}

}  // namespace

TEST(SerialIoWorkerDiagnostics, TimeoutPreservesAbsentResponseAndReason)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->fail_query = true;
  state->injected_error = "serial query timed out before line delimiter";

  const std::string log = runConfigurationFailure(state);

  EXPECT_NE(log.find("received=\"<no response>\""), std::string::npos);
  EXPECT_NE(log.find("serial query timed out before line delimiter"), std::string::npos);
}

TEST(SerialIoWorkerStartup, DrainsStartupBannerBeforeSingleStartupStop)
{
  auto state = std::make_shared<FakeTransportState>();
  state->startup_drain_result = {
    true, std::string("\0\0Starting ...\r", 15), "", true};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();

  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  std::this_thread::sleep_for(30ms);

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    ASSERT_EQ(state->startup_drains.size(), 1u);
    EXPECT_EQ(state->startup_drains.front().raw_bytes, std::string("\0\0Starting ...\r", 15));
    EXPECT_EQ(state->queries, std::vector<std::string>{"?FID\r"});
  }
  const auto batches = writeBatches(state);
  ASSERT_EQ(batches.size(), 1u);
  EXPECT_TRUE(isStopBatch(batches.front()));
  worker.stop();
}

TEST(SerialIoWorkerStartup, ExactHardwareFirmwareIdReachesWaitingForFreshCommand)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "?FID\r";
  state->injected_response = "FID=Roboteq v1.8d SBL2XXX 1/8/2018";
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();

  const auto status = waitForWorkerState(
    worker, driver::SerialConnectionState::waiting_for_fresh_command);

  EXPECT_EQ(status.connection_state, driver::SerialConnectionState::waiting_for_fresh_command);
  EXPECT_EQ(status.framing_state, driver::SerialFramingState::synchronized);
  EXPECT_EQ(status.connection_generation, 1u);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    EXPECT_EQ(state->startup_drains.size(), 1u);
    EXPECT_EQ(state->queries, std::vector<std::string>{"?FID\r"});
    EXPECT_EQ(state->close_calls, 0);
  }
  EXPECT_FALSE(worker.isReadyForMotion());
  worker.stop();
}

TEST(SerialIoWorkerStartup, StartupValidationFailureDoesNotIssueRepeatedFailureStop)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "?FID\r";
  state->fail_query = true;
  state->injected_error = "injected startup FID failure";
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.reconnect_interval = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();

  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (worker.status().connection_state != driver::SerialConnectionState::unhealthy &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(worker.status().connection_state, driver::SerialConnectionState::unhealthy);
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::unresolved);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    ASSERT_EQ(state->write_batches.size(), 1u);
    EXPECT_TRUE(isStopBatch(state->write_batches.front()));
    EXPECT_EQ(state->queries, std::vector<std::string>{"?FID\r"});
    EXPECT_EQ(state->close_calls, 1);
  }
  worker.stop();
}

TEST(SerialIoWorkerStartup, FailedStartupDrainBlocksStartupWithoutStopBatch)
{
  auto state = std::make_shared<FakeTransportState>();
  state->startup_drain_result = {
    false, "Starting ...", "startup input ended with a partial line", false};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.reconnect_interval = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();

  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (worker.status().connection_state != driver::SerialConnectionState::unhealthy &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(worker.status().connection_state, driver::SerialConnectionState::unhealthy);
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::unresolved);
  EXPECT_TRUE(writeBatches(state).empty());
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    ASSERT_EQ(state->startup_drains.size(), 1u);
    EXPECT_TRUE(state->queries.empty());
  }
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, TimeoutBecomesUnknownAndRecoversExactlyOnce)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "FF=", "diagnostic query timed out", {}};
  state->recovery_result = {true, "0\r", "FS=0", ""};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  ASSERT_TRUE(waitForRecoveryCalls(state, 1));
  ASSERT_EQ(
    waitForFramingState(worker, driver::SerialFramingState::synchronized).
    framing_state,
    driver::SerialFramingState::synchronized);
  const auto telemetry = worker.latestDiagnosticTelemetry();
  ASSERT_TRUE(telemetry.has_value());
  EXPECT_FALSE(telemetry->valid);
  EXPECT_EQ(telemetry->connection_generation, 1u);
  EXPECT_NE(telemetry->failure_reason.find("timed out"), std::string::npos);
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::synchronized);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    ASSERT_EQ(state->recovery_calls, 1);
    ASSERT_EQ(state->synchronization_transactions.size(), 1u);
    EXPECT_EQ(state->synchronization_transactions.front().command, "?FS\r");
    EXPECT_EQ(state->synchronization_transactions.front().expected_prefix, "FS=");
  }
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, SuccessfulQueryProducesGenerationTaggedTelemetry)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_generation, 1u);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (!worker.latestDiagnosticTelemetry().has_value() &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  const auto telemetry = worker.latestDiagnosticTelemetry();
  ASSERT_TRUE(telemetry.has_value());
  EXPECT_TRUE(telemetry->valid);
  EXPECT_EQ(telemetry->raw_value, "FF=0\r");
  EXPECT_EQ(telemetry->connection_generation, 1u);
  EXPECT_TRUE(telemetry->failure_reason.empty());
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, StatusTimeoutUsesFaultFlagsForStatusSynchronization)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::status_flags));
  ASSERT_TRUE(waitForRecoveryCalls(state, 1));
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    ASSERT_EQ(state->synchronization_transactions.size(), 1u);
    EXPECT_EQ(state->synchronization_transactions.front().command, "?FF\r");
  }
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, PendingRuntimeStopPrecedesRecovery)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_diagnostic_result = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->diagnostic_entered;}));
  }
  worker.requestStop();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_diagnostic_result = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(waitForRecoveryCalls(state, 1));
  const auto batches = writeBatches(state);
  ASSERT_GE(batches.size(), 2u);
  EXPECT_EQ(
    batches[1],
    (std::vector<std::string>{"!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"}));
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, NewestMotionWinsDiagnosticClaimWindow)
{
  auto state = std::make_shared<FakeTransportState>();
  state->hold_encoder_query = true;
  auto config = workerConfig();
  config.encoder_poll_period = 10ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->encoder_query_entered;}));
  }
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  worker.submitCommand(0.3, 0.4);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_encoder_query = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(waitForCommand(state, "!S 1 18\r"));
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    bool diagnostic_started = false;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      diagnostic_started = !state->diagnostic_transactions.empty();
    }
    if (diagnostic_started) {
      break;
    }
    std::this_thread::sleep_for(1ms);
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto motion = std::find(state->events.begin(), state->events.end(), "write:!S 1 18\r");
    const auto diagnostic = std::find(
      state->events.begin(), state->events.end(), "diagnostic:?FF\r");
    ASSERT_NE(motion, state->events.end());
    ASSERT_NE(diagnostic, state->events.end());
    EXPECT_LT(motion, diagnostic);
  }
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, NewNonzeroMotionCannotPreemptUnresolvedRecovery)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_diagnostic_result = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->diagnostic_entered;}));
  }
  worker.submitCommand(0.3, 0.4);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_diagnostic_result = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(waitForRecoveryCalls(state, 1));
  ASSERT_TRUE(waitForCommand(state, "!S 1 18\r"));
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto recovery = std::find(state->events.begin(), state->events.end(), "recovery");
    const auto motion = std::find(state->events.begin(), state->events.end(), "write:!S 1 18\r");
    ASSERT_NE(recovery, state->events.end());
    ASSERT_NE(motion, state->events.end());
    EXPECT_LT(recovery, motion);
  }
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, PendingZeroCommandDoesNotStarveRecovery)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_diagnostic_result = true;
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->diagnostic_entered;}));
  }

  worker.submitCommand(0.0, 0.0);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_diagnostic_result = true;
    state->cv.notify_all();
  }
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->recovery_entered;}));
  }

  EXPECT_EQ(writeBatches(state).size(), 1u);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  ASSERT_EQ(
    waitForFramingState(worker, driver::SerialFramingState::synchronized).framing_state,
    driver::SerialFramingState::synchronized);
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, NonzeroCommandWaitsUntilFramingIsSynchronized)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->recovery_entered;}));
  }
  worker.submitCommand(0.3, 0.4);
  EXPECT_FALSE(worker.isReadyForMotion());
  EXPECT_EQ(writeBatches(state).size(), 1u);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(waitForCommand(state, "!S 1 18\r"));
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, FailedRecoveryReconnectsAndRejectsOldStateAndCommand)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->recovery_result = {false, "FF=0\rFS=0\r", "", "ambiguous concatenated reply"};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.reconnect_interval = 10ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_generation, 1u);
  worker.submitCommand(0.2, 0.2);
  ASSERT_TRUE(waitForCommand(state, "!S 1 12\r"));
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));

  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (worker.status().connection_generation < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(worker.status().connection_generation, 2u);
  EXPECT_EQ(
    worker.status().connection_state, driver::SerialConnectionState::waiting_for_fresh_command);
  const auto telemetry = worker.latestDiagnosticTelemetry();
  ASSERT_TRUE(telemetry.has_value());
  EXPECT_FALSE(telemetry->valid);
  EXPECT_EQ(telemetry->connection_generation, 1u);
  EXPECT_EQ(state->recovery_calls, 1);
  std::this_thread::sleep_for(20ms);
  const auto batches = writeBatches(state);
  EXPECT_EQ(
    std::count(
      batches.begin(), batches.end(),
      std::vector<std::string>{"!S 1 12\r", "!S 2 12\r"}),
    1);
  worker.stop();
}

TEST(SerialIoWorkerDiagnosticRecovery, ReconnectFailureRemainsFailClosed)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->recovery_result = {false, "FF=", "", "partial delayed reply"};
  state->fail_open_after = 1;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.reconnect_interval = 10ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  ASSERT_TRUE(waitForRecoveryCalls(state, 1));
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(worker.isConnected());
  EXPECT_FALSE(worker.isReadyForMotion());
  EXPECT_EQ(worker.status().connection_generation, 1u);
  EXPECT_EQ(state->recovery_calls, 1);
  worker.stop();
}

TEST(SerialIoWorkerDiagnostics, ControllerRejectionPreservesRawResponseAndReason)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->fail_query = true;
  state->injected_response = "-";
  state->injected_error = "Roboteq rejected query";

  const std::string log = runConfigurationFailure(state);

  EXPECT_NE(log.find("received=\"-\""), std::string::npos);
  EXPECT_NE(log.find("reason=Roboteq rejected query"), std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, MalformedResponseIsObservable)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->injected_response = "KP=not-an-integer";

  const std::string log = runConfigurationFailure(state);

  EXPECT_NE(log.find("received=\"KP=not-an-integer\""), std::string::npos);
  EXPECT_NE(log.find("category=malformed_response"), std::string::npos);
  EXPECT_NE(
    log.find("reason=malformed numeric response: value is not an integer"),
    std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, WrongPrefixIsObservable)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->injected_response = "KI=1";

  const std::string log = runConfigurationFailure(state);

  EXPECT_NE(log.find("expected_prefix=\"KP=\""), std::string::npos);
  EXPECT_NE(log.find("received=\"KI=1\""), std::string::npos);
  EXPECT_NE(log.find("category=wrong_prefix"), std::string::npos);
  EXPECT_NE(log.find("reason=wrong response prefix"), std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, ValueMismatchIncludesExpectedAndActual)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->injected_response = "KP=8";

  const std::string log = runConfigurationFailure(state, 7);

  EXPECT_NE(log.find("expected_value=\"7\""), std::string::npos);
  EXPECT_NE(log.find("reason=value mismatch; expected=7 actual=8"), std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, TransportExceptionPreservesCompleteDiagnosticContext)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->throw_query_exception = true;
  state->injected_error = "injected transport exception";

  const std::string log = runConfigurationFailure(state, 7);

  EXPECT_NE(
    log.find(
      "connectAndValidate: configuration validation failed: phase=configuration_validation "
      "category=transport_exception query_name=KP channel=1"),
    std::string::npos);
  EXPECT_NE(log.find("transmitted=\"~KP 1\\r\""), std::string::npos);
  EXPECT_NE(log.find("expected_prefix=\"KP=\" expected_value=\"7\""), std::string::npos);
  EXPECT_NE(log.find("received=\"<no response>\""), std::string::npos);
  EXPECT_NE(
    log.find("reason=transport exception: injected transport exception"),
    std::string::npos);
  EXPECT_NE(
    log.find("connection_state_transition=configuring->unhealthy; reconnect scheduled"),
    std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, CommunicationValidationPreservesTransportError)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "?FID\r";
  state->fail_query = true;
  state->injected_error = "serial query timed out waiting for FID=";

  const std::string log = runConfigurationFailure(state, 1, false);

  EXPECT_NE(
    log.find(
      "connectAndValidate: communication validation failed: phase=communication_validation "
      "category=timeout query_name=FID channel=none"),
    std::string::npos);
  EXPECT_NE(log.find("transmitted=\"?FID\\r\""), std::string::npos);
  EXPECT_NE(log.find("expected_value=\"non-empty firmware identifier\""), std::string::npos);
  EXPECT_NE(log.find("serial query timed out waiting for FID="), std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, DiagnosticTextEscapesControlAndDelimiterCharacters)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "~KP 1\r";
  state->injected_response = std::string("bad\"\\\t\0\x01", 8);

  const std::string log = runConfigurationFailure(state);

  EXPECT_NE(log.find("received=\"bad\\\"\\\\\\t\\x00\\x01\""), std::string::npos);
}

TEST(SerialIoWorkerDiagnostics, EmptyFirmwareIdentifierHasDetailedParserReason)
{
  auto state = std::make_shared<FakeTransportState>();
  state->failing_query = "?FID\r";
  state->injected_response = "FID=";

  const std::string log = runConfigurationFailure(state, 1, false);

  EXPECT_NE(log.find("category=malformed_response"), std::string::npos);
  EXPECT_NE(
    log.find("reason=malformed firmware response: identifier is empty"),
    std::string::npos);
}

TEST(SerialIoWorker, OnlyWorkerThreadAccessesTransport)
{
  auto state = std::make_shared<FakeTransportState>();
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), workerConfig());

  const auto test_thread = std::this_thread::get_id();
  worker.start();
  ASSERT_TRUE(waitForWriteBatches(state, 1));
  worker.stop();

  std::lock_guard<std::mutex> lock(state->mutex);
  ASSERT_FALSE(state->transport_thread_ids.empty());
  for (const auto & id : state->transport_thread_ids) {
    EXPECT_NE(id, test_thread);
  }
}

TEST(SerialIoWorker, StatusAndReadinessUseCachedTransportSnapshot)
{
  auto state = std::make_shared<FakeTransportState>();
  state->forbidden_is_open_thread = std::this_thread::get_id();
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), workerConfig());

  worker.start();
  const auto connected_status = waitForWorkerState(
    worker, driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(connected_status.transport_open);
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(worker.status().transport_open);
    EXPECT_TRUE(worker.isConnected());
  }
  worker.stop();

  EXPECT_EQ(state->forbidden_is_open_calls.load(), 0);
  EXPECT_FALSE(worker.status().transport_open);
}

TEST(SerialIoWorkerReadiness, DisconnectedIsNotConnectedOrMotionReady)
{
  auto state = std::make_shared<FakeTransportState>();
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), workerConfig());

  const auto status = worker.status();

  EXPECT_EQ(status.connection_state, driver::SerialConnectionState::disconnected);
  EXPECT_FALSE(worker.isConnected());
  EXPECT_FALSE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);
}

TEST(SerialIoWorkerReadiness, ConnectedConfiguringIsNotMotionReady)
{
  auto state = std::make_shared<FakeTransportState>();
  state->query_delay = 200ms;
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  const auto status = waitForWorkerState(worker, driver::SerialConnectionState::configuring);
  worker.stop();

  ASSERT_EQ(status.connection_state, driver::SerialConnectionState::configuring);
  EXPECT_TRUE(status.transport_open);
  EXPECT_FALSE(status.ready_for_motion);
}

TEST(SerialIoWorkerReadiness, WaitingForFreshCommandIsConnectedButNotMotionReady)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  const auto status = waitForWorkerState(
    worker, driver::SerialConnectionState::waiting_for_fresh_command);

  EXPECT_EQ(status.connection_state, driver::SerialConnectionState::waiting_for_fresh_command);
  EXPECT_TRUE(worker.isConnected());
  EXPECT_FALSE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);
  worker.stop();
}

TEST(SerialIoWorkerReadiness, FreshCommandTransitionsWaitingWorkerToMotionReady)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.submitCommand(0.5, 0.5);
  const auto status = waitForWorkerState(worker, driver::SerialConnectionState::ready);

  EXPECT_EQ(status.connection_state, driver::SerialConnectionState::ready);
  EXPECT_TRUE(worker.isConnected());
  EXPECT_TRUE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);
  worker.stop();
}

TEST(SerialIoWorkerReadiness, FullyReadyWithoutFreshCommandGateIsMotionReady)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  config.require_fresh_command_after_reconnect = false;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  const auto status = waitForWorkerState(worker, driver::SerialConnectionState::ready);

  EXPECT_EQ(status.connection_state, driver::SerialConnectionState::ready);
  EXPECT_TRUE(worker.isConnected());
  EXPECT_TRUE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);
  worker.stop();
}

TEST(SerialIoWorker, SubmitCommandDoesNotBlockDuringSlowSerialWrite)
{
  auto state = std::make_shared<FakeTransportState>();
  state->write_delay = 250ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), workerConfig());

  worker.start();
  std::this_thread::sleep_for(10ms);
  const auto start = std::chrono::steady_clock::now();
  worker.submitCommand(1.0, 1.0);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  worker.stop();

  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50);
}

TEST(SerialIoWorker, PollsEncodersIntoLatestSample)
{
  auto state = std::make_shared<FakeTransportState>();
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), workerConfig());

  worker.start();
  std::optional<driver::EncoderSample> sample;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    sample = worker.takeLatestEncoderSample();
    if (sample.has_value()) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }
  worker.stop();

  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->valid);
  EXPECT_EQ(sample->channel_1, 10);
  EXPECT_EQ(sample->channel_2, 20);
}

TEST(SerialIoWorkerDiagnostics, EncoderPollUpdatesBoundedStatusSnapshot)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  const auto connected_status = waitForWorkerState(
    worker, driver::SerialConnectionState::waiting_for_fresh_command);
  driver::SerialWorkerStatus encoder_status;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    encoder_status = worker.status();
    if (encoder_status.latest_encoder_sequence > 0) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }
  worker.stop();

  EXPECT_TRUE(hasEncoderQuery(state));
  EXPECT_GT(encoder_status.latest_encoder_sequence, 0u);
  EXPECT_GT(encoder_status.update_sequence, connected_status.update_sequence);
}

TEST(SerialIoWorker, LatestCommandWinsWhileWorkerIsBusy)
{
  auto state = std::make_shared<FakeTransportState>();
  state->write_delay = 80ms;
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  config.require_fresh_command_after_reconnect = false;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  worker.submitCommand(1.0, 1.0);
  worker.submitCommand(0.5, 0.5);
  ASSERT_TRUE(waitForCommand(state, "!S 1 30\r", 1000ms));
  worker.stop();

  const auto batches = writeBatches(state);
  EXPECT_FALSE(
    std::any_of(
      batches.begin(),
      batches.end(),
      [](const auto & batch) {
        return std::find(batch.begin(), batch.end(), "!S 1 60\r") != batch.end();
      }));
}

TEST(SerialIoWorker, MalformedEncoderResponseIsNotPublishedAsValidSample)
{
  auto state = std::make_shared<FakeTransportState>();
  state->encoder_response = "CR=bad:data";
  auto config = workerConfig();
  config.encoder_poll_period = 10ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  std::this_thread::sleep_for(80ms);
  const auto sample = worker.takeLatestEncoderSample();
  worker.stop();

  EXPECT_FALSE(sample.has_value());
}

TEST(SerialIoWorkerRuntimeStop, CoalescesAndRejectsCommandsSubmittedBeforeCompletion)
{
  auto state = std::make_shared<FakeTransportState>();
  auto events = std::make_shared<ValidationEventCollector>();
  state->hold_encoder_query = true;
  auto config = workerConfig();
  config.encoder_poll_period = 10ms;
  config.command_timeout = 1000ms;
  config.stop_request_observer = [events](const driver::StopRequestEvent & event) {
      events->observeStop(event);
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->encoder_query_entered;}));
  }
  const auto first = worker.requestStop();
  const auto second = worker.requestStop();
  EXPECT_EQ(first, second);
  worker.submitCommand(0.5, 0.5);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_encoder_query = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(events->waitForStopAccepted());
  std::this_thread::sleep_for(20ms);

  const auto before_fresh_command = writeBatches(state);
  ASSERT_EQ(
    std::count_if(
      before_fresh_command.begin(), before_fresh_command.end(), isStopBatch),
    2);  // startup and one coalesced continuing-runtime stop
  EXPECT_FALSE(
    std::any_of(
      before_fresh_command.begin(), before_fresh_command.end(), [](const auto & batch) {
        return std::find(batch.begin(), batch.end(), "!S 1 30\r") != batch.end();
      }));
  worker.submitCommand(0.25, 0.25);
  ASSERT_TRUE(waitForCommand(state, "!S 1 15\r"));
  worker.stop();

  std::lock_guard<std::mutex> lock(events->mutex);
  ASSERT_GE(events->stop_events.size(), 4u);
  EXPECT_EQ(events->stop_events[0].phase, driver::StopRequestPhase::requested);
  EXPECT_EQ(events->stop_events[1].phase, driver::StopRequestPhase::coalesced);
  EXPECT_EQ(events->stop_events[2].phase, driver::StopRequestPhase::write_started);
  EXPECT_EQ(events->stop_events[3].phase, driver::StopRequestPhase::write_accepted);
  EXPECT_EQ(events->stop_events[3].byte_count, 28u);
}

TEST(SerialIoWorkerRuntimeStop, AckCollectionCoalescesStopAndBlocksQueriesAndEncoder)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 5ms;
  config.command_timeout = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->hold_command_ack_collection = true;
    state->encoder_query_entered = false;
  }
  const auto first = worker.requestStop();
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(
      state->cv.wait_for(lock, 500ms, [&]() {return state->command_ack_collection_entered;}));
  }
  const auto second = worker.requestStop();
  EXPECT_EQ(first, second);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    const auto baseline_queries = state->queries.size();
    EXPECT_FALSE(state->cv.wait_for(lock, 25ms, [&]() {return state->encoder_query_entered;}));
    EXPECT_EQ(state->queries.size(), baseline_queries);
    EXPECT_TRUE(state->diagnostic_transactions.empty());
    state->release_command_ack_collection = true;
    state->cv.notify_all();
  }
  const auto diagnostic_deadline = std::chrono::steady_clock::now() + 500ms;
  while (!worker.latestDiagnosticTelemetry().has_value() &&
    std::chrono::steady_clock::now() < diagnostic_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(worker.latestDiagnosticTelemetry().has_value());
  const auto batches = writeBatches(state);
  EXPECT_EQ(std::count_if(batches.begin(), batches.end(), isStopBatch), 2);
  worker.stop();
}

TEST(SerialIoWorkerRuntimeStop, DrainCheckpointStopsBeforeSingleSynchronizationAttempt)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->recovery_entered;}));
  }
  worker.requestStop();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  ASSERT_TRUE(waitForWriteBatches(state, 2));
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto recovery = std::find(state->events.begin(), state->events.end(), "recovery");
    ASSERT_NE(recovery, state->events.end());
    const auto stop_write = std::find(recovery, state->events.end(), "write:!G 1 0\r");
    const auto synchronization = std::find(recovery, state->events.end(), "synchronization");
    ASSERT_NE(stop_write, state->events.end());
    ASSERT_NE(synchronization, state->events.end());
    EXPECT_LT(stop_write, synchronization);
    EXPECT_EQ(state->recovery_calls, 1);
  }
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::synchronized);
  worker.stop();
}

TEST(SerialIoWorkerRuntimeStop, PendingRuntimeStopBlocksRecoveryEntryUntilStopCompletes)
{
  auto state = std::make_shared<FakeTransportState>();
  state->diagnostic_result = {
    driver::DiagnosticTransportStatus::timeout, "", "", "diagnostic query timed out", {}};
  state->hold_diagnostic_result = true;
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->diagnostic_entered;}));
  }
  std::size_t write_batches_before_stop = 0;
  std::size_t events_before_stop = 0;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    write_batches_before_stop = state->write_batches.size();
    events_before_stop = state->events.size();
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->hold_write = true;
    state->write_entered = false;
    state->release_write = false;
    state->release_diagnostic_result = true;
    state->cv.notify_all();
  }
  worker.requestStop();
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->write_entered;}));
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_write = true;
    state->cv.notify_all();
  }
  const auto runtime_stop_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < runtime_stop_deadline) {
    bool runtime_stop_recorded = false;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      runtime_stop_recorded = state->write_batches.size() >= write_batches_before_stop + 1;
    }
    if (runtime_stop_recorded) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->recovery_entered;}));
    ASSERT_GE(state->write_batches.size(), write_batches_before_stop + 1);
    ASSERT_GE(state->events.size(), events_before_stop + 1);
    const auto runtime_stop = std::find(
      state->events.begin() + static_cast<std::ptrdiff_t>(events_before_stop),
      state->events.end(), "write:!G 1 0\r");
    ASSERT_NE(runtime_stop, state->events.end());
    const auto recovery = std::find(state->events.begin(), state->events.end(), "recovery");
    ASSERT_NE(recovery, state->events.end());
    EXPECT_LT(runtime_stop, recovery);
    EXPECT_EQ(state->recovery_calls, 1);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  worker.stop();
}

TEST(SerialIoWorkerRuntimeStop, ObserverExceptionsDoNotChangeStopOrDiagnosticBehavior)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.stop_request_observer = [](const driver::StopRequestEvent &) {throw 1;};
  config.diagnostic_phase_observer = [](const driver::DiagnosticPhaseEvent &) {throw 1;};
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.requestStop();
  ASSERT_TRUE(waitForWriteBatches(state, 2));
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (!worker.latestDiagnosticTelemetry().has_value() &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(worker.latestDiagnosticTelemetry().has_value());
  EXPECT_TRUE(worker.latestDiagnosticTelemetry()->valid);
  worker.stop();
}

TEST(SerialIoWorkerRuntimeStop, PartialDiagnosticBytesDoNotTriggerSecondStopRequest)
{
  auto state = std::make_shared<FakeTransportState>();
  auto events = std::make_shared<ValidationEventCollector>();
  std::mutex result_mutex;
  std::condition_variable result_cv;
  std::vector<driver::DiagnosticResultEvent> result_events;
  const auto stamp = std::chrono::steady_clock::now();
  state->diagnostic_result = driver::DiagnosticTransactionResult{
    driver::DiagnosticTransportStatus::timeout, "", "FF",
    "diagnostic query timed out with a partial response",
    false, stamp, stamp, stamp, stamp, stamp, stamp};
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.stop_request_observer = [events](const driver::StopRequestEvent & event) {
      events->observeStop(event);
    };
  config.diagnostic_result_observer = [&result_mutex, &result_cv, &result_events](
    const driver::DiagnosticResultEvent & event) {
      {
        std::lock_guard<std::mutex> lock(result_mutex);
        result_events.push_back(event);
      }
      result_cv.notify_all();
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.requestStop();
  ASSERT_TRUE(events->waitForStopAccepted());
  ASSERT_TRUE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  const auto diagnostic_deadline = std::chrono::steady_clock::now() + 500ms;
  while ((!worker.latestDiagnosticTelemetry().has_value() ||
    !worker.status().diagnostic_recovery_pending) &&
    std::chrono::steady_clock::now() < diagnostic_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }

  const auto telemetry = worker.latestDiagnosticTelemetry();
  ASSERT_TRUE(telemetry.has_value());
  EXPECT_FALSE(telemetry->valid);
  EXPECT_EQ(telemetry->raw_value, "FF");
  EXPECT_EQ(telemetry->raw_value.size(), 2u);
  EXPECT_EQ(roboteq_ros2_driver::bytesToHex(telemetry->raw_value), "4646");
  EXPECT_NE(telemetry->failure_reason.find("partial response"), std::string::npos);
  EXPECT_EQ(telemetry->correlation_id, 2u);
  EXPECT_FALSE(telemetry->delimiter_observed);
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::unresolved);
  EXPECT_TRUE(worker.status().diagnostic_recovery_pending);
  {
    std::unique_lock<std::mutex> lock(result_mutex);
    ASSERT_TRUE(result_cv.wait_for(lock, 500ms, [&]() {return !result_events.empty();}));
    ASSERT_EQ(result_events.size(), 1u);
    EXPECT_EQ(result_events.front().raw_bytes, "FF");
    EXPECT_EQ(result_events.front().framing_state, driver::SerialFramingState::unresolved);
    EXPECT_EQ(result_events.front().status, driver::DiagnosticTransportStatus::timeout);
  }

  {
    std::lock_guard<std::mutex> lock(events->mutex);
    const auto accepted_stop_count = std::count_if(
      events->stop_events.begin(), events->stop_events.end(),
      [](const driver::StopRequestEvent & event) {
        return event.phase == driver::StopRequestPhase::write_accepted;
      });
    EXPECT_EQ(accepted_stop_count, 1);
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  worker.stop();
}

TEST(SerialIoWorkerCommandOwnership, UnresolvedBlocksQueriesRecoversAndNeverReplaysMotion)
{
  auto state = std::make_shared<FakeTransportState>();
  state->command_statuses = {
    driver::CommandTransportStatus::success,
    driver::CommandTransportStatus::unresolved,
    driver::CommandTransportStatus::success};
  state->hold_recovery = true;
  auto config = workerConfig();
  config.encoder_poll_period = 5ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.submitCommand(0.5, 0.5);
  ASSERT_TRUE(waitForCommand(state, "!S 1 30\r"));
  ASSERT_TRUE(waitForWriteBatches(state, 3));  // startup, ambiguous motion, safety stop
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    ASSERT_TRUE(state->cv.wait_for(lock, 500ms, [&]() {return state->recovery_entered;}));
    const auto query_count_while_unresolved = state->queries.size();
    const auto batches_while_unresolved = state->write_batches.size();
    lock.unlock();
    worker.submitCommand(0.0, 0.0);
    lock.lock();
    EXPECT_FALSE(
      state->cv.wait_for(
        lock, 25ms, [&]() {
          return state->queries.size() != query_count_while_unresolved;
        }));
    EXPECT_EQ(state->queries.size(), query_count_while_unresolved);
    EXPECT_FALSE(
      state->cv.wait_for(
        lock, 25ms, [&]() {
          return state->write_batches.size() != batches_while_unresolved;
        }));
    EXPECT_EQ(state->write_batches.size(), batches_while_unresolved);
  }
  EXPECT_EQ(worker.status().framing_state, driver::SerialFramingState::unresolved);
  EXPECT_FALSE(worker.queueDiagnosticQuery(driver::DiagnosticQueryKind::fault_flags));
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->release_recovery = true;
    state->cv.notify_all();
  }
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (worker.status().framing_state != driver::SerialFramingState::synchronized &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(worker.status().framing_state, driver::SerialFramingState::synchronized);
  std::this_thread::sleep_for(20ms);
  const auto batches = writeBatches(state);
  const auto matching_motion_batches = std::count_if(
    batches.begin(), batches.end(), [](const auto & batch) {
      return std::find(batch.begin(), batch.end(), "!S 1 30\r") != batch.end();
    });
  EXPECT_EQ(matching_motion_batches, 1);
  worker.stop();
}

TEST(SerialIoWorkerCommandOwnership, AmbiguousRecoveryReconnectsAndInvalidatesGeneration)
{
  auto state = std::make_shared<FakeTransportState>();
  state->command_statuses = {
    driver::CommandTransportStatus::success,
    driver::CommandTransportStatus::unresolved,
    driver::CommandTransportStatus::success};
  state->recovery_result = {false, "+\r", "", "ambiguous delayed acknowledgement"};
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.reconnect_interval = 10ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  const auto generation = worker.status().connection_generation;
  worker.submitCommand(0.5, 0.5);
  const auto reconnect_deadline = std::chrono::steady_clock::now() + 500ms;
  while (worker.status().connection_generation <= generation &&
    std::chrono::steady_clock::now() < reconnect_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GT(worker.status().connection_generation, generation);
  EXPECT_FALSE(worker.isReadyForMotion());
  std::this_thread::sleep_for(20ms);
  const auto batches = writeBatches(state);
  const auto stop_batch_count = std::count_if(
    batches.begin(), batches.end(), [](const auto & batch) {
      return isStopBatch(batch);
    });
  EXPECT_EQ(stop_batch_count, 3);
  const auto motion_batch_count = std::count_if(
    batches.begin(), batches.end(), [](const auto & batch) {
      return std::find(batch.begin(), batch.end(), "!S 1 30\r") != batch.end();
    });
  EXPECT_EQ(motion_batch_count, 1);
  worker.stop();
}

TEST(SerialIoWorkerCommandOwnership, PartialRuntimeStopNeverReportsFullWriteAcceptance)
{
  auto state = std::make_shared<FakeTransportState>();
  state->command_statuses = {
    driver::CommandTransportStatus::success,
    driver::CommandTransportStatus::unresolved,
    driver::CommandTransportStatus::unresolved,
    driver::CommandTransportStatus::unresolved};
  state->command_full_acceptance = {true, false, false, false};
  auto events = std::make_shared<ValidationEventCollector>();
  auto config = workerConfig();
  config.encoder_poll_period = 1000ms;
  config.stop_request_observer = [events](const driver::StopRequestEvent & event) {
      events->observeStop(event);
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  const auto correlation = worker.requestStop();
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  for (;; ) {
    {
      std::lock_guard<std::mutex> lock(events->mutex);
      const auto failed = std::find_if(
        events->stop_events.begin(), events->stop_events.end(), [&](const auto & event) {
          return event.correlation_id == correlation &&
          event.phase == driver::StopRequestPhase::write_failed;
        });
      if (failed != events->stop_events.end()) {
        EXPECT_EQ(failed->byte_count, 0u);
        const auto accepted_count_for_correlation = std::count_if(
          events->stop_events.begin(), events->stop_events.end(), [&](const auto & event) {
            return event.correlation_id == correlation &&
            event.phase == driver::StopRequestPhase::write_accepted;
          });
        EXPECT_EQ(accepted_count_for_correlation, 0);
        break;
      }
    }
    ASSERT_LT(std::chrono::steady_clock::now(), deadline);
    std::this_thread::sleep_for(2ms);
  }
  worker.stop();
}

TEST(SerialIoWorkerCommandOwnership, PartialTimeoutStopReportsWriteFailure)
{
  auto state = std::make_shared<FakeTransportState>();
  state->command_statuses = {
    driver::CommandTransportStatus::success,
    driver::CommandTransportStatus::success,
    driver::CommandTransportStatus::unresolved};
  state->command_full_acceptance = {true, true, false};
  auto events = std::make_shared<TimeoutEventCollector>();
  auto config = workerConfig();
  config.command_timeout = 20ms;
  config.encoder_poll_period = 1000ms;
  config.timeout_stop_observer = [events](const driver::TimeoutStopEvent & event) {
      events->observe(event);
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.submitCommand(0.5, 0.5);
  ASSERT_TRUE(events->waitForCompletion());
  const auto snapshot = events->snapshot();
  const auto completed = std::find_if(
    snapshot.begin(), snapshot.end(), [](const auto & event) {
      return event.phase == driver::TimeoutStopEventPhase::zero_write_completed;
    });
  ASSERT_NE(completed, snapshot.end());
  EXPECT_FALSE(completed->write_succeeded);
  worker.stop();
}

TEST(SerialIoWorkerCommandOwnership, StartupUnresolvedAckBlocksAllValidationQueries)
{
  auto state = std::make_shared<FakeTransportState>();
  state->command_statuses.assign(16, driver::CommandTransportStatus::unresolved);
  state->command_full_acceptance.assign(16, true);
  auto config = workerConfig();
  config.reconnect_interval = 5ms;
  config.encoder_poll_period = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);
  worker.start();
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  for (;; ) {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->open_calls >= 2) {
        EXPECT_TRUE(state->queries.empty());
        break;
      }
    }
    ASSERT_LT(std::chrono::steady_clock::now(), deadline);
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_FALSE(worker.isReadyForMotion());
  worker.stop();
}

TEST(SerialIoWorkerStopLatency, StartupAndShutdownStopsCompleteWithinFakeTransportBound)
{
  auto state = std::make_shared<FakeTransportState>();
  constexpr auto injected_delay = 20ms;
  state->write_delay = injected_delay;
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_TRUE(waitForWriteBatches(state, 1));
  worker.stop();

  const auto observations = writeObservations(state);
  ASSERT_EQ(observations.size(), 2u);
  ASSERT_TRUE(isStopBatch(observations.front().commands));
  ASSERT_TRUE(observations.front().succeeded);
  expectBoundedFakeWriteLatency(observations.front(), injected_delay);
  ASSERT_TRUE(isStopBatch(observations.back().commands));
  ASSERT_TRUE(observations.back().succeeded);
  expectBoundedFakeWriteLatency(observations.back(), injected_delay);
}

TEST(SerialIoWorkerStopLatency, TimeoutEmitsExactlyOneBoundedStopBeforeShutdown)
{
  auto state = std::make_shared<FakeTransportState>();
  auto timeout_events = std::make_shared<TimeoutEventCollector>();
  constexpr auto injected_delay = 10ms;
  state->write_delay = injected_delay;
  auto config = workerConfig();
  config.command_timeout = 40ms;
  config.encoder_poll_period = 50ms;
  config.timeout_stop_observer = [timeout_events](const driver::TimeoutStopEvent & event) {
      timeout_events->observe(event);
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.submitCommand(1.0, 1.0);
  const auto timed_out_sequence = worker.commandSequence();
  ASSERT_TRUE(timeout_events->waitForCompletion(1000ms));

  const auto before_shutdown = writeObservations(state);
  const auto events = timeout_events->snapshot();
  ASSERT_EQ(before_shutdown.size(), 3u);
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].phase, driver::TimeoutStopEventPhase::timeout_detected);
  EXPECT_EQ(events[1].phase, driver::TimeoutStopEventPhase::zero_write_started);
  EXPECT_EQ(events[2].phase, driver::TimeoutStopEventPhase::zero_write_completed);
  EXPECT_EQ(events[0].command_sequence, timed_out_sequence);
  EXPECT_EQ(events[1].command_sequence, timed_out_sequence);
  EXPECT_EQ(events[2].command_sequence, timed_out_sequence);
  EXPECT_LE(events[0].timestamp, events[1].timestamp);
  EXPECT_LE(events[1].timestamp, events[2].timestamp);
  EXPECT_TRUE(events[2].write_succeeded);
  EXPECT_TRUE(isStopBatch(before_shutdown[0].commands));
  EXPECT_FALSE(isStopBatch(before_shutdown[1].commands));
  EXPECT_TRUE(isStopBatch(before_shutdown[2].commands));
  const auto stop_batches_after_motion = std::count_if(
    before_shutdown.begin() + 1, before_shutdown.end(),
    [](const auto & observation) {return isStopBatch(observation.commands);});
  EXPECT_EQ(stop_batches_after_motion, 1);
  EXPECT_TRUE(before_shutdown[2].succeeded);
  expectBoundedFakeWriteLatency(before_shutdown[2], injected_delay);
  EXPECT_LE(events[1].timestamp, before_shutdown[2].started);
  EXPECT_LE(before_shutdown[2].completed, events[2].timestamp);
  EXPECT_TRUE(
    std::all_of(
      before_shutdown.begin(), before_shutdown.end(), [&events](const auto & observation) {
        return observation.started < events[0].timestamp || isStopBatch(observation.commands);
      }));
  const auto detection_to_zero_write_completion = events[2].timestamp - events[0].timestamp;
  EXPECT_GE(detection_to_zero_write_completion, injected_delay);
  EXPECT_LT(detection_to_zero_write_completion, 250ms);

  std::this_thread::sleep_for(config.command_timeout * 2);
  const auto after_stale_window = writeObservations(state);
  ASSERT_EQ(after_stale_window.size(), 3u);
  EXPECT_FALSE(
    std::any_of(
      after_stale_window.begin() + 2, after_stale_window.end(), [](const auto & observation) {
        return !isStopBatch(observation.commands);
      }));

  worker.stop();
}

TEST(SerialIoWorkerStopLatency, NormalCommandEmitsNoTimeoutObservation)
{
  auto state = std::make_shared<FakeTransportState>();
  auto timeout_events = std::make_shared<TimeoutEventCollector>();
  auto config = workerConfig();
  config.command_timeout = 500ms;
  config.encoder_poll_period = 500ms;
  config.timeout_stop_observer = [timeout_events](const driver::TimeoutStopEvent & event) {
      timeout_events->observe(event);
    };
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  worker.submitCommand(1.0, 1.0);
  ASSERT_TRUE(waitForCommand(state, "!S 1 60\r"));
  worker.stop();

  EXPECT_TRUE(timeout_events->snapshot().empty());
}

TEST(SerialIoWorkerStopLatency, FailedMotionWriteIsFollowedByBoundedStopBeforeReconnect)
{
  auto state = std::make_shared<FakeTransportState>();
  constexpr auto injected_delay = 10ms;
  state->write_delay = injected_delay;
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  config.reconnect_interval = 1000ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->fail_next_write = true;
  }
  worker.submitCommand(1.0, 1.0);
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::unhealthy).connection_state,
    driver::SerialConnectionState::unhealthy);

  const auto before_shutdown = writeObservations(state);
  ASSERT_EQ(before_shutdown.size(), 3u);
  EXPECT_TRUE(isStopBatch(before_shutdown[0].commands));
  EXPECT_FALSE(before_shutdown[1].succeeded);
  EXPECT_FALSE(isStopBatch(before_shutdown[1].commands));
  EXPECT_TRUE(before_shutdown[2].succeeded);
  EXPECT_TRUE(isStopBatch(before_shutdown[2].commands));
  expectBoundedFakeWriteLatency(before_shutdown[2], injected_delay);
  EXPECT_LT(before_shutdown[2].completed - before_shutdown[1].completed, 250ms);

  worker.stop();
}

TEST(SerialIoWorker, TimeoutSendsOneStopAndInvalidatesStaleCommand)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_TRUE(waitForWriteBatches(state, 1));
  worker.submitCommand(1.0, 1.0);
  ASSERT_TRUE(waitForWriteBatches(state, 2));
  std::this_thread::sleep_for(120ms);
  worker.stop();

  const auto batches = writeBatches(state);
  const auto stop_count = std::count_if(batches.begin(), batches.end(), isStopBatch);
  const auto motion_command_count = std::count_if(
    batches.begin(),
    batches.end(),
    [](const auto & batch) {
      return std::find(batch.begin(), batch.end(), "!S 1 60\r") != batch.end();
    });
  EXPECT_GE(stop_count, 3);  // startup, timeout, shutdown
  EXPECT_EQ(motion_command_count, 1);
}

TEST(SerialIoWorker, ReconnectRequiresFreshCommandAfterFailure)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_TRUE(waitForWriteBatches(state, 1));
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->fail_next_write = true;
  }
  worker.submitCommand(1.0, 1.0);
  std::this_thread::sleep_for(120ms);
  worker.submitCommand(0.5, 0.5);
  ASSERT_TRUE(waitForCommand(state, "!S 1 30\r"));
  worker.stop();

  const auto batches = writeBatches(state);
  EXPECT_TRUE(
    std::any_of(
      batches.begin(),
      batches.end(),
      [](const auto & batch) {
        return std::find(batch.begin(), batch.end(), "!S 1 30\r") != batch.end();
      }));
}

TEST(SerialIoWorkerReadiness, ReconnectReturnsToWaitingBeforeFreshCommandCanResumeMotion)
{
  auto state = std::make_shared<FakeTransportState>();
  auto config = workerConfig();
  config.encoder_poll_period = 500ms;
  driver::SerialIoWorker worker(std::make_unique<FakeTransport>(state), config);

  worker.start();
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::waiting_for_fresh_command).
    connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->fail_next_write = true;
  }
  worker.submitCommand(1.0, 1.0);
  ASSERT_EQ(
    waitForWorkerState(worker, driver::SerialConnectionState::unhealthy).connection_state,
    driver::SerialConnectionState::unhealthy);
  const auto reconnected_status = waitForWorkerState(
    worker, driver::SerialConnectionState::waiting_for_fresh_command);

  EXPECT_EQ(
    reconnected_status.connection_state,
    driver::SerialConnectionState::waiting_for_fresh_command);
  EXPECT_TRUE(worker.isConnected());
  EXPECT_FALSE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);

  worker.submitCommand(0.5, 0.5);
  const auto ready_status = waitForWorkerState(worker, driver::SerialConnectionState::ready);

  EXPECT_EQ(ready_status.connection_state, driver::SerialConnectionState::ready);
  EXPECT_TRUE(worker.isReadyForMotion());
  expectReadinessConsistent(worker);
  worker.stop();
}
