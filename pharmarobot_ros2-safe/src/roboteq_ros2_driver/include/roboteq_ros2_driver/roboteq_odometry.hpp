#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_

#include <optional>
#include <string>

#include "differential_drive_kinematics.hpp"
#include "roboteq_ros2_driver/odom_twist.hpp"

namespace roboteq_ros2_driver
{
namespace odometry
{

struct WheelTickDelta
{
  int left_ticks;
  int right_ticks;
};

struct IntegrationResult
{
  WheelTickDelta ticks;
  RobotPose pose;
  odom_twist::MeasuredTwist twist;
};

std::optional<WheelTickDelta> map_channel_encoder_sample_to_wheels(
  int channel_1_ticks,
  int channel_2_ticks,
  const std::string & channel_1,
  const std::string & channel_2,
  int encoder_sign_1 = -1,
  int encoder_sign_2 = -1);

class OdometryIntegrator
{
public:
  void init(
    double wheel_radius,
    double wheelbase,
    int encoder_cpr);

  std::optional<IntegrationResult> integrate_channel_sample(
    int channel_1_ticks,
    int channel_2_ticks,
    double dt,
    const std::string & channel_1,
    const std::string & channel_2,
    int encoder_sign_1 = -1,
    int encoder_sign_2 = -1);

private:
  DifferentialDriveKinematics kinematics_;
  RobotPose current_pose_{0.0, 0.0, 0.0};
  bool twist_initialized_{false};
};

}  // namespace odometry
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_
