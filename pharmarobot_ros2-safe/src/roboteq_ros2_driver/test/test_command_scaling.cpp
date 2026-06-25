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

#include <cmath>
#include <limits>

#include "roboteq_ros2_driver/command_scaling.hpp"

namespace scaling = roboteq_ros2_driver::command_scaling;

TEST(CommandScaling, LeavesZeroCommandUnchanged)
{
  const auto scaled = scaling::scale_pair_to_limit(0.0, 0.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 0.0);
  EXPECT_DOUBLE_EQ(scaled.second, 0.0);
}

TEST(CommandScaling, LeavesUnderLimitCommandUnchanged)
{
  const auto scaled = scaling::scale_pair_to_limit(25.0, -50.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 25.0);
  EXPECT_DOUBLE_EQ(scaled.second, -50.0);
}

TEST(CommandScaling, ScalesPureTranslationEqually)
{
  const auto scaled = scaling::scale_pair_to_limit(150.0, 150.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 100.0);
  EXPECT_DOUBLE_EQ(scaled.second, 100.0);
}

TEST(CommandScaling, ScalesPureRotationEqually)
{
  const auto scaled = scaling::scale_pair_to_limit(150.0, -150.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 100.0);
  EXPECT_DOUBLE_EQ(scaled.second, -100.0);
}

TEST(CommandScaling, ScalesMixedCommandAndPreservesRatio)
{
  const auto scaled = scaling::scale_pair_to_limit(200.0, 100.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 100.0);
  EXPECT_DOUBLE_EQ(scaled.second, 50.0);
}

TEST(CommandScaling, PreservesNegativeCommandSigns)
{
  const auto scaled = scaling::scale_pair_to_limit(-200.0, -100.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, -100.0);
  EXPECT_DOUBLE_EQ(scaled.second, -50.0);
}

TEST(CommandScaling, ScalesBothWhenOneWheelExceedsLimit)
{
  const auto scaled = scaling::scale_pair_to_limit(80.0, 160.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, 50.0);
  EXPECT_DOUBLE_EQ(scaled.second, 100.0);
}

TEST(CommandScaling, ScalesBothWhenBothWheelsExceedLimit)
{
  const auto scaled = scaling::scale_pair_to_limit(-300.0, 200.0, 100.0);

  EXPECT_DOUBLE_EQ(scaled.first, -100.0);
  EXPECT_NEAR(scaled.second, 100.0 * 2.0 / 3.0, 1e-12);
}

TEST(CommandScaling, ReturnsZeroForInvalidLimit)
{
  const auto zero_limit = scaling::scale_pair_to_limit(10.0, 20.0, 0.0);
  const auto negative_limit = scaling::scale_pair_to_limit(10.0, 20.0, -1.0);

  EXPECT_DOUBLE_EQ(zero_limit.first, 0.0);
  EXPECT_DOUBLE_EQ(zero_limit.second, 0.0);
  EXPECT_DOUBLE_EQ(negative_limit.first, 0.0);
  EXPECT_DOUBLE_EQ(negative_limit.second, 0.0);
}

TEST(CommandScaling, ReturnsZeroForNonFiniteInputOrLimit)
{
  const auto non_finite_input = scaling::scale_pair_to_limit(
    std::numeric_limits<double>::infinity(), 20.0, 100.0);
  const auto non_finite_limit = scaling::scale_pair_to_limit(
    10.0, 20.0, std::numeric_limits<double>::infinity());

  EXPECT_DOUBLE_EQ(non_finite_input.first, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_input.second, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_limit.first, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_limit.second, 0.0);
}
