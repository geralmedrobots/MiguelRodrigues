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

#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

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
