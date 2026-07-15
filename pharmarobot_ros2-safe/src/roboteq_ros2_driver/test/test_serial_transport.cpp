// Copyright 2026 Medrobots
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
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

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>

#include <atomic>

#include <chrono>
#include <string>
#include <thread>
#include <array>
#include <vector>

#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

class PseudoTerminal
{
public:
  PseudoTerminal()
  : master_(posix_openpt(O_RDWR | O_NOCTTY))
  {
    if (master_ >= 0 && grantpt(master_) == 0 && unlockpt(master_) == 0) {
      const char * name = ptsname(master_);
      if (name != nullptr) {
        slave_name_ = name;
      }
    }
  }

  ~PseudoTerminal()
  {
    if (master_ >= 0) {
      close(master_);
    }
  }

  bool valid() const
  {
    return master_ >= 0 && !slave_name_.empty();
  }

  int master() const
  {
    return master_;
  }

  const std::string & slaveName() const
  {
    return slave_name_;
  }

private:
  int master_{-1};
  std::string slave_name_;
};

driver::SerialTransportConfig transportConfig(const std::string & port)
{
  driver::SerialTransportConfig config;
  config.port = port;
  config.read_timeout = 10ms;
  config.write_timeout = 10ms;
  config.transaction_timeout = 50ms;
  config.max_response_bytes = 64;
  return config;
}

ssize_t writeReply(int descriptor, const std::string & reply)
{
  std::this_thread::sleep_for(5ms);
  return ::write(descriptor, reply.data(), reply.size());
}

void replyToSynchronizationQuery(
  int descriptor, const std::string & delayed, const std::string & synchronization_reply)
{
  if (!delayed.empty()) {
    std::this_thread::sleep_for(2ms);
    (void)::write(descriptor, delayed.data(), delayed.size());
  }
  std::array<char, 32> query{};
  (void)::read(descriptor, query.data(), query.size());
  (void)::write(descriptor, synchronization_reply.data(), synchronization_reply.size());
}

void replyThenSendLateBytes(int descriptor)
{
  std::array<char, 32> query{};
  (void)::read(descriptor, query.data(), query.size());
  const std::string reply = "FS=1\r";
  (void)::write(descriptor, reply.data(), reply.size());
  std::this_thread::sleep_for(8ms);
  const std::string late = "X";
  (void)::write(descriptor, late.data(), late.size());
}

void completeReplyAfterTimeoutThenSynchronize(int descriptor)
{
  std::this_thread::sleep_for(5ms);
  (void)::write(descriptor, "FF", 2);
  std::this_thread::sleep_for(25ms);
  (void)::write(descriptor, "=0\r", 3);
  std::array<char, 32> query{};
  (void)::read(descriptor, query.data(), query.size());
  (void)::write(descriptor, "FS=1\r", 5);
}

driver::DiagnosticRecoveryBounds shortRecoveryBounds()
{
  driver::DiagnosticRecoveryBounds bounds;
  bounds.delayed_reply_horizon = 0ms;
  bounds.drain_absolute_limit = 30ms;
  bounds.drain_quiet_period = 5ms;
  bounds.synchronization_timeout = 20ms;
  bounds.post_sync_absolute_limit = 20ms;
  bounds.post_sync_quiet_period = 5ms;
  return bounds;
}

void replyToCommandBatch(int descriptor, std::size_t command_bytes, const std::string & reply)
{
  std::string received(command_bytes, '\0');
  std::size_t offset = 0;
  while (offset < received.size()) {
    const auto count = ::read(descriptor, received.data() + offset, received.size() - offset);
    if (count <= 0) {
      return;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (!reply.empty()) {
    (void)::write(descriptor, reply.data(), reply.size());
  }
}

driver::CommandTransactionBounds shortCommandBounds()
{
  driver::CommandTransactionBounds bounds;
  bounds.acknowledgement_deadline = 40ms;
  bounds.post_ack_quiet_period = 5ms;
  bounds.max_response_bytes = 64;
  return bounds;
}

}  // namespace

TEST(SerialTransportStartupDrain, DrainsCompleteStartupBannerBeforeCommandOwnership)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const std::string banner{std::string("\0\0Starting ...\r", 15)};
  ASSERT_EQ(::write(terminal.master(), banner.data(), banner.size()), 15);

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 40ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);

  EXPECT_TRUE(result.synchronized);
  EXPECT_EQ(result.raw_bytes, banner);
  EXPECT_TRUE(result.delimiter_observed);
  EXPECT_TRUE(result.reason.empty());
}

TEST(SerialTransportStartupDrain, WaitsForDelayedStartupBannerBeforeQuietCompletion)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const std::string banner{std::string("\0\0Starting ...\r", 15)};

  std::thread delayed_banner([&]() {
      std::this_thread::sleep_for(30ms);
      (void)::write(terminal.master(), banner.data(), banner.size());
    });

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 180ms;
  bounds.quiet_period = 80ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);
  delayed_banner.join();

  EXPECT_TRUE(result.synchronized);
  EXPECT_EQ(result.raw_bytes, banner);
  EXPECT_TRUE(result.delimiter_observed);
  EXPECT_TRUE(result.reason.empty());
}

TEST(SerialTransportStartupDrain, PartialStartupInputFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const std::string partial = "Starting ...";
  ASSERT_EQ(::write(terminal.master(), partial.data(), partial.size()), 12);

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 40ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);

  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ(result.raw_bytes, partial);
  EXPECT_FALSE(result.delimiter_observed);
  EXPECT_NE(result.reason.find("partial line"), std::string::npos);
}

TEST(SerialTransportStartupDrain, SplitBannerAcrossReadsRemainsOwnedByDrain)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  std::thread fragmented_banner([&]() {
      const std::string part_1{std::string("\0\0Sta", 5)};
      const std::string part_2{"rting ..."};
      const std::string part_3{"\r"};
      (void)::write(terminal.master(), part_1.data(), part_1.size());
      std::this_thread::sleep_for(2ms);
      (void)::write(terminal.master(), part_2.data(), part_2.size());
      std::this_thread::sleep_for(2ms);
      (void)::write(terminal.master(), part_3.data(), part_3.size());
    });

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 80ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);
  fragmented_banner.join();

  EXPECT_TRUE(result.synchronized);
  EXPECT_EQ(result.raw_bytes, std::string("\0\0Starting ...\r", 15));
  EXPECT_TRUE(result.delimiter_observed);
  EXPECT_TRUE(result.reason.empty());
}

TEST(SerialTransportStartupDrain, NoStartupInputCompletesOnQuietBoundary)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 40ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);

  EXPECT_TRUE(result.synchronized);
  EXPECT_TRUE(result.raw_bytes.empty());
  EXPECT_FALSE(result.delimiter_observed);
  EXPECT_TRUE(result.reason.empty());
}

TEST(SerialTransportStartupDrain, MalformedStartupInputFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  const std::string malformed{std::string("\0\0Sta\x01rting ...\r", 16)};
  ASSERT_EQ(::write(terminal.master(), malformed.data(), malformed.size()), 16);

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 40ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 64;
  const auto result = transport.drainStartupInput(bounds);

  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ(result.raw_bytes, malformed);
  EXPECT_TRUE(result.delimiter_observed);
  EXPECT_NE(result.reason.find("malformed"), std::string::npos);
}

TEST(SerialTransportStartupDrain, OversizedStartupInputFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  const std::string oversized(80, 'A');
  ASSERT_EQ(::write(terminal.master(), oversized.data(), oversized.size()), 80);

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 40ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 32;
  const auto result = transport.drainStartupInput(bounds);

  EXPECT_FALSE(result.synchronized);
  EXPECT_GT(result.raw_bytes.size(), bounds.max_bytes);
  EXPECT_NE(result.reason.find("maximum size"), std::string::npos);
}

TEST(SerialTransportStartupDrain, ContinuousInputPreventsQuietBoundary)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  std::atomic<bool> keep_writing{true};
  std::thread noisy_stream([&]() {
      while (keep_writing.load()) {
        (void)::write(terminal.master(), "A", 1);
        std::this_thread::sleep_for(1ms);
      }
    });

  driver::StartupDrainBounds bounds;
  bounds.absolute_limit = 25ms;
  bounds.quiet_period = 5ms;
  bounds.max_bytes = 512;
  const auto result = transport.drainStartupInput(bounds);

  keep_writing = false;
  noisy_stream.join();

  EXPECT_FALSE(result.synchronized);
  EXPECT_FALSE(result.raw_bytes.empty());
  EXPECT_NE(result.reason.find("absolute deadline"), std::string::npos);
}

TEST(SerialTransportCommandOwnership, OwnsFourConcatenatedAcknowledgements)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const std::vector<std::string> commands{
    "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
  std::thread controller(replyToCommandBatch, terminal.master(), 28u, "+\r+\r+\r+\r");
  const auto result = transport.commandTransaction(commands, shortCommandBounds());
  controller.join();
  EXPECT_EQ(result.status, driver::CommandTransportStatus::success) << result.reason;
  EXPECT_EQ(result.raw_bytes, "+\r+\r+\r+\r");
  EXPECT_EQ(result.received_acknowledgements, 4u);
  EXPECT_NE(result.write_accepted_at, std::chrono::steady_clock::time_point{});
  EXPECT_LE(result.write_accepted_at, result.completed_at);
}

TEST(SerialTransportCommandOwnership, MissingOrPartialAcknowledgementIsUnresolved)
{
  for (const std::string reply : {"", "+\r", "+\r+\r", "+\r+\r+\r", "+\r+\r+\r+"}) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    const std::vector<std::string> commands{
      "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
    std::thread controller(replyToCommandBatch, terminal.master(), 28u, reply);
    const auto result = transport.commandTransaction(commands, shortCommandBounds());
    controller.join();
    EXPECT_EQ(result.status, driver::CommandTransportStatus::unresolved);
  }
}

TEST(SerialTransportCommandOwnership, ExtraAckOrTypedReplyIsNeverDiscarded)
{
  for (const std::string reply : {
    "+\r+\r+\r+\r+\r", "+\r+\rFF=0\r", "+\r+\r+\r+\rFF=0\r"
  })
  {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    const std::vector<std::string> commands{
      "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
    std::thread controller(replyToCommandBatch, terminal.master(), 28u, reply);
    const auto result = transport.commandTransaction(commands, shortCommandBounds());
    controller.join();
    EXPECT_EQ(result.status, driver::CommandTransportStatus::unresolved);
    EXPECT_EQ(result.raw_bytes, reply);
  }
}

TEST(SerialTransportCommandOwnership, QueryRejectsUnownedAcknowledgement)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "+\r");
  std::string response;
  EXPECT_FALSE(transport.query("?CR\r", "CR=", response, error));
  controller.join();
  EXPECT_EQ(response, "+");
  EXPECT_EQ(error, "unowned command acknowledgement encountered during query");
}

TEST(SerialTransportCommandOwnership, QueryObserverExceptionDoesNotChangeResult)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.query_observer = [](const driver::QueryTraceEvent &) {throw 1;};
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "+\r");
  std::string response;

  EXPECT_FALSE(transport.query("?FID\r", "FID=", response, error));

  controller.join();
  EXPECT_EQ(response, "+");
  EXPECT_EQ(error, "unowned command acknowledgement encountered during query");
}

TEST(SerialTransportCommandOwnership, SeparateAndDelayedFourthAckRemainOwned)
{
  for (const bool delay_fourth : {false, true}) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    const std::vector<std::string> commands{
      "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
    std::thread controller([&]() {
        std::array<char, 28> bytes{};
        (void)::read(terminal.master(), bytes.data(), bytes.size());
        for (int index = 0; index < 4; ++index) {
          if (delay_fourth && index == 3) {
            std::this_thread::sleep_for(15ms);
          } else {
            std::this_thread::sleep_for(1ms);
          }
          (void)::write(terminal.master(), "+\r", 2);
        }
      });
    const auto result = transport.commandTransaction(commands, shortCommandBounds());
    controller.join();
    EXPECT_EQ(result.status, driver::CommandTransportStatus::success) << result.reason;
    EXPECT_EQ(result.received_acknowledgements, 4u);
  }
}

TEST(SerialTransportCommandOwnership, LateAckAfterBoundaryContaminatesNextQueryFailClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.post_reply_quiet_period = 5ms;
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const std::vector<std::string> commands{
    "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
  std::thread controller([&]() {
      std::array<char, 28> stop{};
      (void)::read(terminal.master(), stop.data(), stop.size());
      (void)::write(terminal.master(), "+\r+\r+\r+\r", 8);
      std::this_thread::sleep_for(12ms);
      (void)::write(terminal.master(), "+\r", 2);
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      (void)::write(terminal.master(), "CR=1:2\r", 7);
    });
  ASSERT_EQ(
    transport.commandTransaction(commands, shortCommandBounds()).status,
    driver::CommandTransportStatus::success);
  std::this_thread::sleep_for(15ms);
  std::string response;
  EXPECT_FALSE(transport.query("?CR\r", "CR=", response, error));
  controller.join();
  EXPECT_EQ(response, "+");
  EXPECT_EQ(error, "unowned command acknowledgement encountered during query");
}

TEST(SerialTransportCommandOwnership, MalformedAndStartupLinesAreUnresolved)
{
  for (const std::string reply : {"OK\r", "Roboteq startup\r", "FF=0\r+\r"}) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    const std::vector<std::string> commands{"!G 1 0\r"};
    std::thread controller(replyToCommandBatch, terminal.master(), 7u, reply);
    const auto result = transport.commandTransaction(commands, shortCommandBounds());
    controller.join();
    EXPECT_EQ(result.status, driver::CommandTransportStatus::unresolved);
    EXPECT_FALSE(result.raw_bytes.empty());
  }
}

TEST(SerialTransportCommandOwnership, QueryExpectedReplyFollowedByAckIsUnresolved)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller([&]() {
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      (void)::write(terminal.master(), "CR=1:2\r", 7);
      std::this_thread::sleep_for(5ms);
      (void)::write(terminal.master(), "+\r", 2);
    });
  std::string response;
  EXPECT_FALSE(transport.query("?CR\r", "CR=", response, error));
  controller.join();
  EXPECT_NE(error.find("extra bytes"), std::string::npos);
  EXPECT_NE(response.find("+\r"), std::string::npos);
}

TEST(SerialTransportDiagnostics, ValidFirmwareReplySucceedsWithoutTrailingBytes)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(
    writeReply, terminal.master(), "FID=Roboteq v1.8d SBL2XXX 1/8/2018\r");

  std::string response;
  EXPECT_TRUE(transport.query("?FID\r", "FID=", response, error)) << error;
  controller.join();

  EXPECT_EQ(response, "FID=Roboteq v1.8d SBL2XXX 1/8/2018");
  EXPECT_TRUE(error.empty());
}

TEST(SerialTransportDiagnostics, ValidFaultFlagsReplySucceedsWithoutTrailingBytes)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "FF=0\r");

  std::string response;
  EXPECT_TRUE(transport.query("?FF\r", "FF=", response, error)) << error;
  controller.join();

  EXPECT_EQ(response, "FF=0");
  EXPECT_TRUE(error.empty());
}

TEST(SerialTransportDiagnostics, ValidReplyNearDeadlineStillSucceedsWhenNoTrailingBytesArrive)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.transaction_timeout = 15ms;
  config.post_reply_quiet_period = 20ms;
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller([&]() {
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      std::this_thread::sleep_for(12ms);
      (void)::write(terminal.master(), "FID=Roboteq v1.8d SBL2XXX 1/8/2018\r", 35);
    });

  std::string response;
  EXPECT_TRUE(transport.query("?FID\r", "FID=", response, error)) << error;
  controller.join();

  EXPECT_EQ(response, "FID=Roboteq v1.8d SBL2XXX 1/8/2018");
  EXPECT_TRUE(error.empty());
}

TEST(SerialTransportDiagnostics, LateTrailingAckInsidePostReplyQuietWindowFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.transaction_timeout = 15ms;
  config.post_reply_quiet_period = 20ms;
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller([&]() {
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      std::this_thread::sleep_for(12ms);
      (void)::write(terminal.master(), "FID=Roboteq v1.8d SBL2XXX 1/8/2018\r", 35);
      std::this_thread::sleep_for(8ms);
      (void)::write(terminal.master(), "+\r", 2);
    });

  std::string response;
  EXPECT_FALSE(transport.query("?FID\r", "FID=", response, error));
  controller.join();

  EXPECT_NE(error.find("extra bytes after query response"), std::string::npos);
  EXPECT_NE(response.find("+\r"), std::string::npos);
}

TEST(SerialTransportDiagnostics, ValidReplyFollowedByUnexpectedCompleteLineFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.post_reply_quiet_period = 20ms;
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller([&]() {
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      (void)::write(terminal.master(), "FF=0\rOK\r", 8);
    });

  std::string response;
  EXPECT_FALSE(transport.query("?FF\r", "FF=", response, error));
  controller.join();

  EXPECT_EQ(response, "FF=0; extra_raw=OK\r");
  EXPECT_NE(error.find("extra bytes after query response"), std::string::npos);
}

TEST(SerialTransportDiagnostics, ValidReplyFollowedByPartialTrailingBytesFailsClosed)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  auto config = transportConfig(terminal.slaveName());
  config.post_reply_quiet_period = 20ms;
  driver::RoboteqSerialTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller([&]() {
      std::array<char, 16> query{};
      (void)::read(terminal.master(), query.data(), query.size());
      (void)::write(terminal.master(), "FF=0\rX", 6);
    });

  std::string response;
  EXPECT_FALSE(transport.query("?FF\r", "FF=", response, error));
  controller.join();

  EXPECT_EQ(response, "FF=0; extra_raw=X");
  EXPECT_NE(error.find("extra bytes after query response"), std::string::npos);
}

TEST(SerialTransportDiagnostics, RejectionReturnsRawControllerReply)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "-\r");

  std::string response;
  EXPECT_FALSE(transport.query("~KP 1\r", "KP=", response, error));
  controller.join();

  EXPECT_EQ(response, "-");
  EXPECT_EQ(error, "Roboteq rejected query");
}

TEST(SerialTransportDiagnostics, WrongPrefixPreservesSubsequentTimeoutReason)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "KI=1\r");

  std::string response;
  EXPECT_FALSE(transport.query("~KP 1\r", "KP=", response, error));
  controller.join();

  EXPECT_EQ(response, "KI=1");
  EXPECT_EQ(error, "unexpected response line during query: KI=1");
}

TEST(SerialTransportDiagnostics, PartialMalformedReplyPreservesBytesAndTimeout)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "KP=abc");

  std::string response;
  EXPECT_FALSE(transport.query("~KP 1\r", "KP=", response, error));
  controller.join();

  EXPECT_EQ(response, "KP=abc");
  EXPECT_EQ(error, "serial query timed out before line delimiter");
}

TEST(SerialTransportDiagnostics, NoExpectedReplyBeforeDeadlineFails)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;

  std::string response;
  EXPECT_FALSE(transport.query("?FF\r", "FF=", response, error));

  EXPECT_TRUE(response.empty());
  EXPECT_EQ(error, "serial query timed out waiting for FF=");
}

TEST(SerialTransportDiagnosticRecovery, StrictQueryAcceptsOnlyCompleteExpectedResponse)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(writeReply, terminal.master(), "FF=0\r");
  const auto result = transport.diagnosticQuery({"?FF\r", "FF=", 30ms});
  controller.join();
  EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::success);
  EXPECT_EQ(result.response, "FF=0");
  EXPECT_EQ(result.raw_bytes, "FF=0\r");
}

TEST(SerialTransportDiagnosticRecovery, ReportsMonotonicCorrelatedObservableBoundaries)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::vector<driver::DiagnosticPhaseEvent> events;
  driver::DiagnosticTransaction transaction{"?FF\r", "FF=", 30ms};
  transaction.correlation_id = 42;
  transaction.connection_generation = 7;
  transaction.observer = [&events](const driver::DiagnosticPhaseEvent & event) {
      events.push_back(event);
    };
  std::thread controller(writeReply, terminal.master(), "FF=0\r");
  const auto result = transport.diagnosticQuery(transaction);
  controller.join();

  ASSERT_EQ(result.status, driver::DiagnosticTransportStatus::success);
  EXPECT_NE(result.started_at, std::chrono::steady_clock::time_point{});
  EXPECT_LE(result.started_at, result.write_accepted_at);
  EXPECT_LE(result.write_accepted_at, result.first_byte_at);
  EXPECT_LE(result.first_byte_at, result.completed_at);
  ASSERT_GE(events.size(), 5u);
  EXPECT_EQ(events.front().phase, driver::DiagnosticPhase::write_started);
  EXPECT_EQ(events.back().phase, driver::DiagnosticPhase::transaction_complete);
  for (std::size_t index = 0; index < events.size(); ++index) {
    EXPECT_EQ(events[index].correlation_id, 42u);
    EXPECT_EQ(events[index].connection_generation, 7u);
    if (index > 0) {
      EXPECT_LE(events[index - 1].timestamp, events[index].timestamp);
    }
  }
}

TEST(SerialTransportDiagnosticRecovery, WrongPrefixAndMalformedPayloadFailClosed)
{
  for (const std::string reply : {"FS=0\r", "FF=abc\r", "FF=0\rFS=0\r"}) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    std::thread controller(writeReply, terminal.master(), reply);
    const auto result = transport.diagnosticQuery({"?FF\r", "FF=", 30ms});
    controller.join();
    EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::failure);
    EXPECT_TRUE(
      result.reason.find("malformed") != std::string::npos ||
      result.reason.find("extra bytes") != std::string::npos);
  }
}

TEST(SerialTransportDiagnosticRecovery, PartialAndOversizedResponsesFailClosed)
{
  {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    std::thread controller(writeReply, terminal.master(), "FF=1");
    const auto result = transport.diagnosticQuery({"?FF\r", "FF=", 20ms});
    controller.join();
    EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::timeout);
    EXPECT_EQ(result.raw_bytes, "FF=1");
  }
  {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    auto config = transportConfig(terminal.slaveName());
    config.max_response_bytes = 6;
    driver::RoboteqSerialTransport transport(config);
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    std::thread controller(writeReply, terminal.master(), "FF=12345\r");
    const auto result = transport.diagnosticQuery({"?FF\r", "FF=", 30ms});
    controller.join();
    EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::failure);
    EXPECT_NE(result.reason.find("maximum size"), std::string::npos);
  }
}

TEST(SerialTransportDiagnosticRecovery, TwoBytePartialResponseRecordsFirstByteAndTimeout)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::vector<driver::DiagnosticPhaseEvent> events;
  driver::DiagnosticTransaction transaction{"?FF\r", "FF=", 20ms};
  transaction.observer = [&events](const driver::DiagnosticPhaseEvent & event) {
      events.push_back(event);
    };
  std::thread controller(writeReply, terminal.master(), "FF");
  const auto result = transport.diagnosticQuery(transaction);
  controller.join();

  EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::timeout);
  EXPECT_EQ(result.raw_bytes, "FF");
  EXPECT_EQ(result.reason, "diagnostic query timed out with a partial response");
  EXPECT_NE(result.first_byte_at, std::chrono::steady_clock::time_point{});
  EXPECT_NE(result.last_byte_at, std::chrono::steady_clock::time_point{});
  EXPECT_NE(result.timeout_at, std::chrono::steady_clock::time_point{});
  EXPECT_FALSE(result.delimiter_observed);
  EXPECT_EQ(result.raw_bytes.size(), 2u);
  ASSERT_GE(events.size(), 5u);
  EXPECT_EQ(events[0].phase, driver::DiagnosticPhase::write_started);
  EXPECT_EQ(events[1].phase, driver::DiagnosticPhase::write_accepted);
  EXPECT_EQ(events[2].phase, driver::DiagnosticPhase::waiting_for_first_byte);
  EXPECT_EQ(events[3].phase, driver::DiagnosticPhase::first_byte_received);
  EXPECT_EQ(events[4].phase, driver::DiagnosticPhase::timeout_or_unresolved);
  EXPECT_EQ(events[4].byte_count, 2u);
}

TEST(SerialTransportDiagnosticRecovery, DelayedCompletionAfterTimeoutFailsClosedWithoutStitching)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(completeReplyAfterTimeoutThenSynchronize, terminal.master());
  const auto initial = transport.diagnosticQuery({"?FF\r", "FF=", 20ms});
  EXPECT_EQ(initial.status, driver::DiagnosticTransportStatus::timeout);
  EXPECT_EQ(initial.raw_bytes, "FF");

  const auto recovered = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 20ms}, initial.started_at,
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(), {});
  controller.join();

  EXPECT_FALSE(recovered.synchronized);
  EXPECT_EQ(recovered.drained_raw_bytes, "=0\r");
  EXPECT_EQ(recovered.synchronization_response, "=0");
  EXPECT_NE(recovered.reason.find("malformed"), std::string::npos);
}

TEST(SerialTransportDiagnosticRecovery, RejectsAnyControllerWriteOutsideReadOnlyAllowlist)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  const auto result = transport.diagnosticQuery({"!G 1 100\r", "G=", 20ms});
  EXPECT_EQ(result.status, driver::DiagnosticTransportStatus::failure);
  EXPECT_NE(result.reason.find("allowlist"), std::string::npos);
}

TEST(SerialTransportDiagnosticRecovery, CompleteDelayedReplyIsDrainedBeforeDistinctSync)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(
    replyToSynchronizationQuery, terminal.master(), "FF=0\r", "FS=1\r");
  const auto result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(), {});
  controller.join();
  EXPECT_TRUE(result.synchronized) << result.reason;
  EXPECT_EQ(result.drained_raw_bytes, "FF=0\r");
  EXPECT_EQ(result.synchronization_response, "FS=1");
}

TEST(SerialTransportDiagnosticRecovery, NoDelayedBytesCanStillSynchronizeUnambiguously)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  std::thread controller(
    replyToSynchronizationQuery, terminal.master(), "", "FS=1\r");
  const auto result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(), {});
  controller.join();
  EXPECT_TRUE(result.synchronized) << result.reason;
  EXPECT_TRUE(result.drained_raw_bytes.empty());
}

TEST(SerialTransportDiagnosticRecovery, CheckpointFailurePreventsSynchronizationWrite)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  bool checkpoint_called = false;
  const auto result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(),
    [&](std::string & checkpoint_error) {
      checkpoint_called = true;
      checkpoint_error = "injected stop write failure";
      return false;
    });
  EXPECT_TRUE(checkpoint_called);
  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ(result.reason, "injected stop write failure");
  EXPECT_NE(result.drain_started_at, std::chrono::steady_clock::time_point{});
  EXPECT_LE(result.drain_started_at, result.drain_completed_at);
  EXPECT_LE(result.drain_completed_at, result.completed_at);
}

TEST(SerialTransportDiagnosticRecovery, PartialOrConcatenatedDrainIsAmbiguous)
{
  for (const std::string delayed : {
    "FF=0", "FF=0\rFS=1\r", "\rFF=0\r", "FF=0\r\r"
  })
  {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    ASSERT_EQ(
      ::write(terminal.master(), delayed.data(), delayed.size()),
      static_cast<ssize_t>(delayed.size()));
    const auto result = transport.boundedDiagnosticRecovery(
      {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
      {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(), {});
    EXPECT_FALSE(result.synchronized);
    EXPECT_NE(result.reason.find("partial, malformed, or ambiguous"), std::string::npos);
  }
}

TEST(SerialTransportDiagnosticRecovery, DrainAndSynchronizationDeadlinesAreFinite)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  auto bounds = shortRecoveryBounds();
  bounds.drain_absolute_limit = 5ms;
  bounds.drain_quiet_period = 20ms;
  const auto drain_result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 10ms}, bounds, {});
  EXPECT_FALSE(drain_result.synchronized);
  EXPECT_NE(drain_result.reason.find("absolute deadline"), std::string::npos);

  bounds = shortRecoveryBounds();
  bounds.synchronization_timeout = 5ms;
  const auto sync_result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 5ms}, bounds, {});
  EXPECT_FALSE(sync_result.synchronized);
  EXPECT_NE(sync_result.reason.find("timed out"), std::string::npos);
}

TEST(SerialTransportDiagnosticRecovery, MalformedOrExtraSynchronizationDataFailsClosed)
{
  for (const std::string reply : {"FS=abc\r", "FS=1\rFF=0\r"}) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
    std::string error;
    ASSERT_TRUE(transport.open(error)) << error;
    std::thread controller(
      replyToSynchronizationQuery, terminal.master(), "", reply);
    const auto result = transport.boundedDiagnosticRecovery(
      {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
      {"?FS\r", "FS=", 20ms}, shortRecoveryBounds(), {});
    controller.join();
    EXPECT_FALSE(result.synchronized);
  }
}

TEST(SerialTransportDiagnosticRecovery, PostSyncDeadlineFailurePreservesRawBytes)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  driver::RoboteqSerialTransport transport(transportConfig(terminal.slaveName()));
  std::string error;
  ASSERT_TRUE(transport.open(error)) << error;
  auto bounds = shortRecoveryBounds();
  bounds.synchronization_timeout = 5ms;
  bounds.post_sync_absolute_limit = 10ms;
  bounds.post_sync_quiet_period = 20ms;
  std::thread controller(replyThenSendLateBytes, terminal.master());
  const auto result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 5ms}, bounds, {});
  controller.join();
  EXPECT_FALSE(result.synchronized);
  EXPECT_NE(result.reason.find("post-synchronization framing check failed"), std::string::npos);
  EXPECT_NE(result.drained_raw_bytes.find('X'), std::string::npos);
}
