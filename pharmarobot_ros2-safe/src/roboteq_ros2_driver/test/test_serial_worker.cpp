#include <gtest/gtest.h>

#include <chrono>
#include <algorithm>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

struct FakeTransportState
{
  mutable std::mutex mutex;
  bool open{false};
  bool fail_next_write{false};
  std::string encoder_response{"CR=10:20"};
  int open_calls{0};
  int close_calls{0};
  std::chrono::milliseconds write_delay{0};
  std::chrono::milliseconds query_delay{0};
  std::vector<std::vector<std::string>> write_batches;
  std::vector<std::string> queries;
  std::vector<std::string> logs;
  std::vector<std::thread::id> transport_thread_ids;
  std::string failing_query;
  std::string injected_response;
  std::string injected_error;
  bool fail_query{false};
  bool throw_query_exception{false};
};

class FakeTransport : public driver::IRoboteqSerialTransport
{
public:
  explicit FakeTransport(std::shared_ptr<FakeTransportState> state)
  : state_(std::move(state))
  {
  }

  bool open(std::string &) override
  {
    recordThread();
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->open = true;
    state_->open_calls++;
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
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->open;
  }

  bool sendCommands(const std::vector<std::string> & commands, std::string & error) override
  {
    recordThread();
    if (state_->write_delay.count() > 0) {
      std::this_thread::sleep_for(state_->write_delay);
    }

    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->fail_next_write) {
      state_->fail_next_write = false;
      error = "injected write failure";
      return false;
    }
    state_->write_batches.push_back(commands);
    return true;
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
    std::lock_guard<std::mutex> lock(state_->mutex);
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
      response = state_->encoder_response;
      return true;
    }
    response = expected_prefix + "1";
    return true;
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
