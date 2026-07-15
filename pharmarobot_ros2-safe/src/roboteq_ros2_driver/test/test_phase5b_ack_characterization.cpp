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

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "roboteq_ros2_driver/phase5b_ack_characterization_logic.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

driver::ByteObservation observed(
  char value,
  const std::chrono::steady_clock::time_point & timestamp,
  bool after_query_write = false)
{
  return driver::ByteObservation{value, timestamp, after_query_write};
}

}  // namespace

TEST(Phase5bAckCharacterizationLogic, StopBatchBytesRemainFixed)
{
  EXPECT_EQ(driver::stopBatchBytes(), "!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r");
  EXPECT_EQ(driver::zeroCommandBytes(driver::ZeroCommandKind::g1_zero), "!G 1 0\r");
  EXPECT_EQ(driver::zeroCommandBytes(driver::ZeroCommandKind::g2_zero), "!G 2 0\r");
  EXPECT_EQ(driver::zeroCommandBytes(driver::ZeroCommandKind::s1_zero), "!S 1 0\r");
  EXPECT_EQ(driver::zeroCommandBytes(driver::ZeroCommandKind::s2_zero), "!S 2 0\r");
}

TEST(Phase5bAckCharacterizationLogic, H3ModesMapOnlyToAllowedQueries)
{
  EXPECT_EQ(
    driver::queryBytesForMode(driver::AckCharacterizationMode::h3_stop_then_fid),
    "?FID\r");
  EXPECT_EQ(
    driver::queryExpectedPrefixForMode(driver::AckCharacterizationMode::h3_stop_then_fid),
    "FID=");
  EXPECT_EQ(
    driver::queryBytesForMode(driver::AckCharacterizationMode::h3_stop_then_ff),
    "?FF\r");
  EXPECT_EQ(
    driver::queryExpectedPrefixForMode(driver::AckCharacterizationMode::h3_stop_then_ff),
    "FF=");
}

TEST(Phase5bAckCharacterizationLogic, ConcatenatedAckLinesAreCountedSeparately)
{
  const auto base = std::chrono::steady_clock::now();
  const std::vector<driver::ByteObservation> bytes{
    observed('+', base + 1ms),
    observed('\r', base + 2ms),
    observed('+', base + 3ms),
    observed('\r', base + 5ms)};

  const auto analysis = driver::analyzeCapture(bytes);
  ASSERT_EQ(analysis.lines.size(), 2u);
  EXPECT_EQ(analysis.ack_count, 2u);
  EXPECT_TRUE(analysis.delimiter_observed);
  EXPECT_EQ(analysis.unexpected_line_count, 0u);
  ASSERT_EQ(analysis.inter_line_gap_ns.size(), 1u);
  EXPECT_EQ(
    analysis.inter_line_gap_ns.front(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(3ms).count());
}

TEST(Phase5bAckCharacterizationLogic, QueryReplyAndLateAckAreSeparatedByStage)
{
  const auto base = std::chrono::steady_clock::now();
  const std::vector<driver::ByteObservation> bytes{
    observed('+', base + 1ms, false),
    observed('\r', base + 2ms, false),
    observed('F', base + 3ms, true),
    observed('F', base + 4ms, true),
    observed('=', base + 5ms, true),
    observed('0', base + 6ms, true),
    observed('\r', base + 7ms, true),
    observed('+', base + 8ms, true),
    observed('\r', base + 9ms, true)};

  const auto analysis = driver::analyzeCapture(bytes, std::string("FF="));
  ASSERT_EQ(analysis.lines.size(), 3u);
  EXPECT_EQ(analysis.ack_count, 2u);
  EXPECT_EQ(analysis.ack_count_after_query_write, 1u);
  EXPECT_EQ(analysis.expected_query_reply_count, 1u);
  EXPECT_EQ(analysis.unexpected_line_count, 0u);
  EXPECT_FALSE(analysis.trailing_partial.size());
}

TEST(Phase5bAckCharacterizationLogic, PartialTrailingBytesArePreserved)
{
  const auto base = std::chrono::steady_clock::now();
  const std::vector<driver::ByteObservation> bytes{
    observed('+', base + 1ms),
    observed('\r', base + 2ms),
    observed('F', base + 3ms),
    observed('F', base + 4ms),
    observed('=', base + 5ms)};

  const auto analysis = driver::analyzeCapture(bytes, std::string("FF="));
  ASSERT_EQ(analysis.lines.size(), 1u);
  EXPECT_EQ(analysis.ack_count, 1u);
  EXPECT_EQ(analysis.trailing_partial, "FF=");
  EXPECT_TRUE(analysis.delimiter_observed);
}
