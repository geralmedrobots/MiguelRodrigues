#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

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
  std::vector<std::vector<std::string>> write_batches;
  std::vector<std::string> queries;
  std::vector<std::thread::id> transport_thread_ids;
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
    std::string &) override
  {
    recordThread();
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->queries.push_back(command);
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

}  // namespace

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
