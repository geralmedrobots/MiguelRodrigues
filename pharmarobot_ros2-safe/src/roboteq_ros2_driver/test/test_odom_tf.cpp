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
