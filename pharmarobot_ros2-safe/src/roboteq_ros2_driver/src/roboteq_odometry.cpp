#include "roboteq_ros2_driver/roboteq_odometry.hpp"

#include <climits>

namespace roboteq_ros2_driver
{
namespace odometry
{

std::optional<WheelTickDelta> map_channel_encoder_sample_to_wheels(
  int channel_1_ticks,
  int channel_2_ticks,
  const std::string & channel_1,
  const std::string & channel_2)
{
  if (channel_1_ticks == INT_MAX || channel_2_ticks == INT_MAX) {
    return std::nullopt;
  }

  const int corrected_channel_1 = -channel_1_ticks;
  const int corrected_channel_2 = -channel_2_ticks;

  if (channel_1 == "right" && channel_2 == "left") {
    return WheelTickDelta{corrected_channel_2, corrected_channel_1};
  }
  if (channel_1 == "left" && channel_2 == "right") {
    return WheelTickDelta{corrected_channel_1, corrected_channel_2};
  }
  return std::nullopt;
}

void OdometryIntegrator::init(
  double wheel_radius,
  double wheelbase,
  int encoder_cpr)
{
  kinematics_.initParam(wheel_radius, wheelbase, encoder_cpr);
  current_pose_ = RobotPose{0.0, 0.0, 0.0};
  twist_initialized_ = false;
}

std::optional<IntegrationResult> OdometryIntegrator::integrate_channel_sample(
  int channel_1_ticks,
  int channel_2_ticks,
  double dt,
  const std::string & channel_1,
  const std::string & channel_2)
{
  const auto ticks = map_channel_encoder_sample_to_wheels(
    channel_1_ticks, channel_2_ticks, channel_1, channel_2);
  if (!ticks.has_value()) {
    return std::nullopt;
  }

  const RobotDisplacement displacement =
    kinematics_.calculateForwardKinematics(ticks->left_ticks, ticks->right_ticks);
  const double previous_yaw = current_pose_.theta;
  current_pose_ = kinematics_.updateRobotPose(current_pose_, displacement);
  const auto measured_twist = odom_twist::calculate_measured_twist(
    displacement.linear_x,
    previous_yaw,
    current_pose_.theta,
    dt,
    twist_initialized_);
  twist_initialized_ = true;

  return IntegrationResult{*ticks, current_pose_, measured_twist};
}

}  // namespace odometry
}  // namespace roboteq_ros2_driver
