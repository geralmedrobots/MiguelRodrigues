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

#include <limits>

#include "roboteq_ros2_driver/command_watchdog.hpp"

namespace watchdog = roboteq_ros2_driver::command_watchdog;

TEST(CommandWatchdog, DoesNotStopBeforeFirstCommand)
{
  EXPECT_FALSE(watchdog::should_send_timeout_stop(false, false, 10.0, 0.5));
}

TEST(CommandWatchdog, DoesNotRepeatAlreadyLoggedTimeout)
{
  EXPECT_FALSE(watchdog::should_send_timeout_stop(true, true, 10.0, 0.5));
}

TEST(CommandWatchdog, DoesNotStopForFreshCommand)
{
  EXPECT_FALSE(watchdog::should_send_timeout_stop(true, false, 0.1, 0.5));
}

TEST(CommandWatchdog, StopsForExpiredCommand)
{
  EXPECT_TRUE(watchdog::should_send_timeout_stop(true, false, 0.6, 0.5));
}

TEST(CommandWatchdog, DoesNotStopForNonFiniteAge)
{
  EXPECT_FALSE(
    watchdog::should_send_timeout_stop(
      true, false, std::numeric_limits<double>::infinity(), 0.5));
}
