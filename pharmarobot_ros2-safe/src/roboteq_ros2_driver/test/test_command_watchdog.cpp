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
  EXPECT_FALSE(watchdog::should_send_timeout_stop(
    true, false, std::numeric_limits<double>::infinity(), 0.5));
}
