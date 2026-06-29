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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_

#include <math.h>
#include <unistd.h>

#include <climits>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "roboteq_ros2_driver/driver_parameter_validation.hpp"
#include "roboteq_ros2_driver/msg/wheel_ticks.hpp"
#include "roboteq_ros2_driver/odom_covariance.hpp"
#include "roboteq_ros2_driver/roboteq_diagnostics.hpp"
#include "roboteq_ros2_driver/roboteq_odometry.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"


#define PI M_PI
namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
public:
  Roboteq();  // (nodeOptions options?)
  ~Roboteq();

private:
  rclcpp::Time last_cmd_time_;
  bool received_first_cmd_ = false;
  bool command_timeout_logged_ = false;
  mutable std::mutex command_state_mutex_;
  double cmd_timeout_s_ = 0.5;

  roboteq_ros2_driver::odometry::OdometryIntegrator odometry_integrator_;

  rclcpp::TimerBase::SharedPtr command_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  std::unique_ptr<roboteq_ros2_driver::SerialIoWorker> serial_worker_;

  float odom_x{};
  float odom_y{};
  float odom_yaw{};

  std::chrono::steady_clock::time_point odom_last_time{};
  bool odom_last_time_valid_{false};


  // settings
  bool pub_odom_tf{};
  std::string odom_frame{};
  std::string base_frame{};
  std::string cmdvel_topic{};
  std::string odom_topic{};
  std::string port{};
  int baud{};
  bool open_loop{};
  double wheel_radius{};
  double wheelbase{};
  double wheel_circumference{};
  int encoder_ppr{};
  int encoder_cpr{};
  int encoder_eppr{};
  int motor_sign_1{1};
  int motor_sign_2{1};
  int encoder_sign_1{1};
  int encoder_sign_2{1};
  int command_angular_sign{-1};
  double max_amps{};
  int max_rpm{};
  std::string channel_1{};
  std::string channel_2{};
  int serial_read_timeout_ms_{50};
  int serial_write_timeout_ms_{50};
  int serial_transaction_timeout_ms_{100};
  int serial_max_response_bytes_{256};
  double serial_reconnect_interval_s_{1.0};
  int encoder_poll_period_ms_{50};
  bool require_fresh_command_after_reconnect_{true};
  double diagnostics_publish_rate_hz_{1.0};
  double encoder_freshness_warn_s_{0.25};
  double encoder_freshness_error_s_{1.0};
  roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig odom_covariance_config_{};
  // Test different odom msg memory
  // nav_msgs::msg::Odometry odom_msg{};
  nav_msgs::msg::Odometry odom_msg{};
  // geometry_msgs::msg::Twist twist_msg{};


  //
  // cmd_vel subscriber
  //
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

  //
  // odom publisher
  //
  void odom_setup();
  void odom_loop();
  void odom_publish(const roboteq_ros2_driver::odometry::IntegrationResult & integration);
  void publish_ticks(int left_ticks, int right_ticks);

  void update_parameters();
  roboteq_ros2_driver::parameter_validation::DriverParameters validation_parameters() const;
  void initialize_valid_configuration();
  void command_watchdog_loop();
  void diagnostics_loop();
  void start_serial_worker();

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<roboteq_ros2_driver::msg::WheelTicks>::SharedPtr ticks_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::mutex diagnostics_publication_mutex_;
  roboteq_ros2_driver::DiagnosticsPublisherState diagnostics_state_;
};

}  // namespace Roboteq


#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_
