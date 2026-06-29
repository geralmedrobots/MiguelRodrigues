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

#include "roboteq_ros2_driver/odom_tf.hpp"

namespace odom_tf = roboteq_ros2_driver::odom_tf;

TEST(OdomTf, BuildsOdomToBaseTransform)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;
  const double half_pi = std::acos(-1.0) / 2.0;

  const auto transform = odom_tf::build_odom_to_base_transform(
    "odom", "base_link", stamp, 1.25, -0.5, half_pi);

  EXPECT_EQ(transform.header.stamp.sec, 123);
  EXPECT_EQ(transform.header.stamp.nanosec, 456u);
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.25);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -0.5);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.0);
  EXPECT_NEAR(transform.transform.rotation.x, 0.0, 1e-12);
  EXPECT_NEAR(transform.transform.rotation.y, 0.0, 1e-12);
  EXPECT_NEAR(transform.transform.rotation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(transform.transform.rotation.w, std::sqrt(0.5), 1e-12);
}

TEST(OdomTf, PreservesConfiguredFrameIds)
{
  builtin_interfaces::msg::Time stamp;

  const auto transform = odom_tf::build_odom_to_base_transform(
    "custom_odom", "custom_base", stamp, 0.0, 0.0, 0.0);

  EXPECT_EQ(transform.header.frame_id, "custom_odom");
  EXPECT_EQ(transform.child_frame_id, "custom_base");
}
