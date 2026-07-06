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

#include <chrono>
#include <string>
#include <thread>
#include <array>

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

}  // namespace

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
  EXPECT_EQ(
    error,
    "unexpected response prefix: expected KP= received KI=1; "
    "subsequent_error=serial query timed out before line delimiter");
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
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds());
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
    {"?FS\r", "FS=", 20ms}, shortRecoveryBounds());
  controller.join();
  EXPECT_TRUE(result.synchronized) << result.reason;
  EXPECT_TRUE(result.drained_raw_bytes.empty());
}

TEST(SerialTransportDiagnosticRecovery, PartialOrConcatenatedDrainIsAmbiguous)
{
  for (const std::string delayed : {
    "FF=0", "FF=0\rFS=1\r", "\rFF=0\r", "FF=0\r\r"})
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
      {"?FS\r", "FS=", 20ms}, shortRecoveryBounds());
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
    {"?FS\r", "FS=", 10ms}, bounds);
  EXPECT_FALSE(drain_result.synchronized);
  EXPECT_NE(drain_result.reason.find("absolute deadline"), std::string::npos);

  bounds = shortRecoveryBounds();
  bounds.synchronization_timeout = 5ms;
  const auto sync_result = transport.boundedDiagnosticRecovery(
    {"?FF\r", "FF=", 100ms}, std::chrono::steady_clock::now(),
    {"?FS\r", "FS=", 5ms}, bounds);
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
      {"?FS\r", "FS=", 20ms}, shortRecoveryBounds());
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
    {"?FS\r", "FS=", 5ms}, bounds);
  controller.join();
  EXPECT_FALSE(result.synchronized);
  EXPECT_NE(result.reason.find("post-synchronization framing check failed"), std::string::npos);
  EXPECT_NE(result.drained_raw_bytes.find('X'), std::string::npos);
}
