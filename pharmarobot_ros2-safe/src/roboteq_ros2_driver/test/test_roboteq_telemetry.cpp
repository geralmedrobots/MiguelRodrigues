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

#include <string>

#include "roboteq_ros2_driver/roboteq_telemetry.hpp"

namespace driver = roboteq_ros2_driver;

TEST(RoboteqTelemetryParser, ParsesSignedIntegerExactly)
{
  std::string error;
  const auto value = driver::parseMotorTelemetryInteger("M=-10", "M=", error);
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, -10);
  EXPECT_TRUE(error.empty());
}

TEST(RoboteqTelemetryParser, RejectsMissingPrefixAndTrailingData)
{
  std::string error;
  EXPECT_FALSE(driver::parseMotorTelemetryInteger("S=1x", "S=", error).has_value());
  EXPECT_FALSE(driver::parseMotorTelemetryInteger("M=1", "S=", error).has_value());
}

TEST(RoboteqTelemetryParser, RejectsEmptyAndOutOfRangeValues)
{
  std::string error;
  EXPECT_FALSE(driver::parseMotorTelemetryInteger("A=", "A=", error).has_value());
  EXPECT_FALSE(driver::parseMotorTelemetryInteger(
      "P=999999999999999999999999", "P=", error).has_value());
}

TEST(RoboteqTelemetryQueries, UsesSingleExclusiveWorkerQueryPlan)
{
  const auto & queries = driver::motorTelemetryQueries();
  ASSERT_EQ(queries.size(), 13u);
  EXPECT_STREQ(queries.front().command, "?FF\r");
  EXPECT_STREQ(queries[1].command, "?CIS 1\r");
  EXPECT_STREQ(queries[6].command, "?FM 1\r");
  EXPECT_STREQ(queries.back().command, "?FM 2\r");
  for (const auto & query : queries) {
    EXPECT_EQ(query.command[0], '?');
    EXPECT_NE(query.command[std::char_traits<char>::length(query.command) - 1], '\n');
  }
}
