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

#include <signal.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#define _CMDVEL_DEBUG

// #define _VERBOSE


// Define following to enable odom debug output
#define _ODOM_DEBUG


#define ROBOTEQ_CYCLE_PERIOD 50ms  // ms
#define ROBORTEQ_WRITING_TIMEOUT 5  //

#include "roboteq_ros2_driver/command_watchdog.hpp"
#include "roboteq_ros2_driver/driver_parameter_validation.hpp"
#include "roboteq_ros2_driver/odom_covariance.hpp"
#include "roboteq_ros2_driver/odom_tf.hpp"
#include "roboteq_ros2_driver/roboteq_command_conversion.hpp"
#include "roboteq_ros2_driver/roboteq_configuration.hpp"
#include "roboteq_ros2_driver/roboteq_diagnostics.hpp"
#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"
#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"

namespace
{
double sanitize_covariance_parameter(
  const rclcpp::Logger & logger,
  const char * name,
  double value,
  double fallback)
{
  const double sanitized =
    roboteq_ros2_driver::odom_covariance::sanitize_variance(value, fallback);
  if (sanitized != value) {
    RCLCPP_WARN(
      logger,
      "Invalid odometry covariance parameter '%s'=%.6f; using default %.6f",
      name,
      value,
      fallback);
  }
  return sanitized;
}

roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig
sanitize_covariance_config_with_logging(
  const rclcpp::Logger & logger,
  const roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig & config)
{
  const auto defaults = roboteq_ros2_driver::odom_covariance::default_config();
  return {
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_x", config.pose_x, defaults.pose_x),
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_y", config.pose_y, defaults.pose_y),
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_z", config.pose_z, defaults.pose_z),
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_roll", config.pose_roll, defaults.pose_roll),
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_pitch", config.pose_pitch, defaults.pose_pitch),
    sanitize_covariance_parameter(
      logger, "odom_pose_covariance_yaw", config.pose_yaw, defaults.pose_yaw),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_linear_x",
      config.twist_linear_x,
      defaults.twist_linear_x),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_linear_y",
      config.twist_linear_y,
      defaults.twist_linear_y),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_linear_z",
      config.twist_linear_z,
      defaults.twist_linear_z),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_angular_x",
      config.twist_angular_x,
      defaults.twist_angular_x),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_angular_y",
      config.twist_angular_y,
      defaults.twist_angular_y),
    sanitize_covariance_parameter(
      logger,
      "odom_twist_covariance_angular_z",
      config.twist_angular_z,
      defaults.twist_angular_z),
  };
}

void log_diagnostics_records(
  const rclcpp::Logger & logger,
  const std::vector<roboteq_ros2_driver::DiagnosticsLogRecord> & records)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  for (const auto & record : records) {
    if (record.level >= DiagnosticStatus::ERROR) {
      RCLCPP_ERROR(logger, "%s", record.message.c_str());
    } else if (record.level == DiagnosticStatus::WARN) {
      RCLCPP_WARN(logger, "%s", record.message.c_str());
    } else {
      RCLCPP_INFO(logger, "%s", record.message.c_str());
    }
  }
}
}  // namespace

namespace Roboteq
{
Roboteq::Roboteq()
: Node("roboteq_ros2_driver")
// differential_drive_kinematics_(void)
// initialize parameters and variables
{
  pub_odom_tf = this->declare_parameter("pub_odom_tf", false);
  odom_frame = this->declare_parameter("odom_frame", "odom");
  base_frame = this->declare_parameter("base_frame", "base_link");
  cmdvel_topic = this->declare_parameter("cmdvel_topic", "/cmd_vel/safe");
  odom_topic = this->declare_parameter("odom_topic", "odom");
  port = this->declare_parameter("port", "/dev/ttyUSB0");
  baud = this->declare_parameter("baud", 115200);
  open_loop = this->declare_parameter("open_loop", false);
  wheel_radius = this->declare_parameter("wheel_radius", 0.085);    // in meters
  wheelbase = this->declare_parameter("wheelbase", 0.453);    // in meters
  encoder_ppr = this->declare_parameter("encoder_ppr", 1024);
  encoder_cpr = this->declare_parameter("encoder_cpr", 4096);
  const auto default_eppr = -1024;
  encoder_eppr = this->declare_parameter("encoder_eppr", default_eppr);
  motor_sign_1 = this->declare_parameter("motor_sign_1", 1);
  motor_sign_2 = this->declare_parameter("motor_sign_2", 1);
  encoder_sign_1 = this->declare_parameter("encoder_sign_1", 1);
  encoder_sign_2 = this->declare_parameter("encoder_sign_2", 1);
  command_angular_sign = this->declare_parameter("command_angular_sign", -1);
  max_amps = this->declare_parameter("max_amps", 5.0);
  max_rpm = this->declare_parameter("max_rpm", 100);

  channel_1 = this->declare_parameter("channel_1", "right");
  channel_2 = this->declare_parameter("channel_2", "left");
  cmd_timeout_s_ = this->declare_parameter("cmd_timeout_s", 0.5);
  const auto default_covariance = roboteq_ros2_driver::odom_covariance::default_config();
  odom_covariance_config_.pose_x =
    this->declare_parameter("odom_pose_covariance_x", default_covariance.pose_x);
  odom_covariance_config_.pose_y =
    this->declare_parameter("odom_pose_covariance_y", default_covariance.pose_y);
  odom_covariance_config_.pose_z =
    this->declare_parameter("odom_pose_covariance_z", default_covariance.pose_z);
  odom_covariance_config_.pose_roll =
    this->declare_parameter("odom_pose_covariance_roll", default_covariance.pose_roll);
  odom_covariance_config_.pose_pitch =
    this->declare_parameter("odom_pose_covariance_pitch", default_covariance.pose_pitch);
  odom_covariance_config_.pose_yaw =
    this->declare_parameter("odom_pose_covariance_yaw", default_covariance.pose_yaw);
  odom_covariance_config_.twist_linear_x = this->declare_parameter(
    "odom_twist_covariance_linear_x", default_covariance.twist_linear_x);
  odom_covariance_config_.twist_linear_y = this->declare_parameter(
    "odom_twist_covariance_linear_y", default_covariance.twist_linear_y);
  odom_covariance_config_.twist_linear_z = this->declare_parameter(
    "odom_twist_covariance_linear_z", default_covariance.twist_linear_z);
  odom_covariance_config_.twist_angular_x = this->declare_parameter(
    "odom_twist_covariance_angular_x", default_covariance.twist_angular_x);
  odom_covariance_config_.twist_angular_y = this->declare_parameter(
    "odom_twist_covariance_angular_y", default_covariance.twist_angular_y);
  odom_covariance_config_.twist_angular_z = this->declare_parameter(
    "odom_twist_covariance_angular_z", default_covariance.twist_angular_z);
  serial_read_timeout_ms_ = this->declare_parameter("serial_read_timeout_ms", 50);
  serial_write_timeout_ms_ = this->declare_parameter("serial_write_timeout_ms", 50);
  serial_transaction_timeout_ms_ = this->declare_parameter("serial_transaction_timeout_ms", 100);
  serial_max_response_bytes_ = this->declare_parameter("serial_max_response_bytes", 256);
  serial_reconnect_interval_s_ = this->declare_parameter("serial_reconnect_interval_s", 1.0);
  encoder_poll_period_ms_ = this->declare_parameter("encoder_poll_period_ms", 50);
  diagnostics_publish_rate_hz_ = this->declare_parameter("diagnostics_publish_rate_hz", 1.0);
  encoder_freshness_warn_s_ = this->declare_parameter("encoder_freshness_warn_s", 0.25);
  encoder_freshness_error_s_ = this->declare_parameter("encoder_freshness_error_s", 1.0);
  require_fresh_command_after_reconnect_ =
    this->declare_parameter("require_fresh_command_after_reconnect", true);

  update_parameters();
  const auto error = roboteq_ros2_driver::parameter_validation::validate_then_start(
    validation_parameters(), [this]() {initialize_valid_configuration();});
  if (error) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Invalid safety-critical parameter '%s': %s",
      error->parameter.c_str(),
      error->reason.c_str());
    throw std::invalid_argument(
            "Invalid parameter '" + error->parameter + "': " + error->reason);
  }
}

void Roboteq::initialize_valid_configuration()
{
  RCLCPP_INFO(this->get_logger(), "Parameters initialized ...");
  odometry_integrator_.init(wheel_radius, wheelbase, encoder_cpr);


  odom_x = 0.0;
  odom_y = 0.0;
  odom_yaw = 0.0;
  odom_last_time_valid_ = false;

  wheel_circumference = 2 * PI * wheel_radius;


  odom_msg = nav_msgs::msg::Odometry();

  odom_setup();
//
//  odom publisher
//
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 100);
  ticks_publisher_ =
    this->create_publisher<roboteq_ros2_driver::msg::WheelTicks>("wheel_ticks", 100);
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

//
// cmd_vel subscriber
//

  command_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  feedback_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cmdvel_options;
  cmdvel_options.callback_group = command_callback_group_;
  cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    cmdvel_topic,      // topic name
    1,      // QoS history depth
    std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1),
    cmdvel_options);
  using namespace std::chrono_literals;
  command_watchdog_timer_ = this->create_wall_timer(
    ROBOTEQ_CYCLE_PERIOD,
    std::bind(&Roboteq::command_watchdog_loop, this),
    command_callback_group_);
  odom_timer_ = this->create_wall_timer(
    ROBOTEQ_CYCLE_PERIOD,
    std::bind(&Roboteq::odom_loop, this),
    feedback_callback_group_);
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int>(
        std::max(1.0, 1000.0 / std::max(0.001, diagnostics_publish_rate_hz_)))),
    std::bind(&Roboteq::diagnostics_loop, this),
    feedback_callback_group_);
  start_serial_worker();
  // enable modifying params at run-time
  /*
  using namespace std::chrono_literals;

  param_update_timer =
    this->create_wall_timer(1000ms, std::bind(&Roboteq::update_params, this));
  */
}


void Roboteq::update_parameters()
{
  RCLCPP_INFO(this->get_logger(), "Parameters updated ...");
  this->get_parameter("pub_odom_tf", pub_odom_tf);
  this->get_parameter("odom_frame", odom_frame);
  this->get_parameter("base_frame", base_frame);
  this->get_parameter("cmdvel_topic", cmdvel_topic);
  this->get_parameter("odom_topic", odom_topic);
  this->get_parameter("port", port);
  this->get_parameter("baud", baud);
  this->get_parameter("open_loop", open_loop);
  this->get_parameter("wheel_radius", wheel_radius);
  this->get_parameter("wheelbase", wheelbase);
  this->get_parameter("encoder_ppr", encoder_ppr);
  this->get_parameter("encoder_cpr", encoder_cpr);
  this->get_parameter("encoder_eppr", encoder_eppr);
  this->get_parameter("motor_sign_1", motor_sign_1);
  this->get_parameter("motor_sign_2", motor_sign_2);
  this->get_parameter("encoder_sign_1", encoder_sign_1);
  this->get_parameter("encoder_sign_2", encoder_sign_2);
  this->get_parameter("command_angular_sign", command_angular_sign);
  this->get_parameter("max_amps", max_amps);
  this->get_parameter("max_rpm", max_rpm);
  this->get_parameter("channel_1", channel_1);
  this->get_parameter("channel_2", channel_2);
  this->get_parameter("cmd_timeout_s", cmd_timeout_s_);
  this->get_parameter("odom_pose_covariance_x", odom_covariance_config_.pose_x);
  this->get_parameter("odom_pose_covariance_y", odom_covariance_config_.pose_y);
  this->get_parameter("odom_pose_covariance_z", odom_covariance_config_.pose_z);
  this->get_parameter("odom_pose_covariance_roll", odom_covariance_config_.pose_roll);
  this->get_parameter("odom_pose_covariance_pitch", odom_covariance_config_.pose_pitch);
  this->get_parameter("odom_pose_covariance_yaw", odom_covariance_config_.pose_yaw);
  this->get_parameter("odom_twist_covariance_linear_x", odom_covariance_config_.twist_linear_x);
  this->get_parameter("odom_twist_covariance_linear_y", odom_covariance_config_.twist_linear_y);
  this->get_parameter("odom_twist_covariance_linear_z", odom_covariance_config_.twist_linear_z);
  this->get_parameter("odom_twist_covariance_angular_x", odom_covariance_config_.twist_angular_x);
  this->get_parameter("odom_twist_covariance_angular_y", odom_covariance_config_.twist_angular_y);
  this->get_parameter("odom_twist_covariance_angular_z", odom_covariance_config_.twist_angular_z);
  this->get_parameter("serial_read_timeout_ms", serial_read_timeout_ms_);
  this->get_parameter("serial_write_timeout_ms", serial_write_timeout_ms_);
  this->get_parameter("serial_transaction_timeout_ms", serial_transaction_timeout_ms_);
  this->get_parameter("serial_max_response_bytes", serial_max_response_bytes_);
  this->get_parameter("serial_reconnect_interval_s", serial_reconnect_interval_s_);
  this->get_parameter("encoder_poll_period_ms", encoder_poll_period_ms_);
  this->get_parameter("diagnostics_publish_rate_hz", diagnostics_publish_rate_hz_);
  this->get_parameter("encoder_freshness_warn_s", encoder_freshness_warn_s_);
  this->get_parameter("encoder_freshness_error_s", encoder_freshness_error_s_);
  this->get_parameter(
    "require_fresh_command_after_reconnect",
    require_fresh_command_after_reconnect_);
}

roboteq_ros2_driver::parameter_validation::DriverParameters
Roboteq::validation_parameters() const
{
  return {
    port,
    baud,
    wheel_radius,
    wheelbase,
    encoder_ppr,
    encoder_cpr,
    encoder_eppr,
    motor_sign_1,
    motor_sign_2,
    encoder_sign_1,
    encoder_sign_2,
    command_angular_sign,
    max_amps,
    max_rpm,
    cmd_timeout_s_,
    serial_read_timeout_ms_,
    serial_write_timeout_ms_,
    serial_transaction_timeout_ms_,
    serial_max_response_bytes_,
    serial_reconnect_interval_s_,
    encoder_poll_period_ms_,
    diagnostics_publish_rate_hz_,
    channel_1,
    channel_2,
    encoder_freshness_warn_s_,
    encoder_freshness_error_s_,
  };
}

void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  if (!serial_worker_) {
    RCLCPP_ERROR(this->get_logger(), "Rejected cmd_vel because serial worker is not running");
    return;
  }

  if (!std::isfinite(twist_msg->linear.x) || !std::isfinite(twist_msg->angular.z)) {
    RCLCPP_ERROR(this->get_logger(), "Rejected non-finite cmd_vel command");
    serial_worker_->submitCommand(0.0, 0.0);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_state_mutex_);
    last_cmd_time_ = this->now();
    received_first_cmd_ = true;
    command_timeout_logged_ = false;
  }

  const auto wheel_speeds = roboteq_ros2_driver::command_conversion::twist_to_wheel_speeds(
    twist_msg->linear.x,
    twist_msg->angular.z,
    wheelbase,
    command_angular_sign);

  RCLCPP_DEBUG(
    this->get_logger(),
    "CMD_VEL_TO_WHEELS | linear_x=%.3f angular_z=%.3f | left_speed=%.3f right_speed=%.3f",
    twist_msg->linear.x,
    twist_msg->angular.z,
    wheel_speeds.left_mps,
    wheel_speeds.right_mps
  );

  const auto channel_speeds = roboteq_ros2_driver::command_conversion::wheels_to_channels(
    wheel_speeds,
    channel_1,
    channel_2);
  if (!channel_speeds.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Invalid channel configuration");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "WHEELS_TO_CHANNELS | left_speed=%.3f right_speed=%.3f | "
    "channel_1=%s %.3f | channel_2=%s %.3f",
    wheel_speeds.left_mps,
    wheel_speeds.right_mps,
    channel_1.c_str(),
    channel_speeds->channel_1_mps,
    channel_2.c_str(),
    channel_speeds->channel_2_mps
  );

  const auto signed_channel_speeds = roboteq_ros2_driver::command_conversion::apply_motor_signs(
    *channel_speeds, motor_sign_1, motor_sign_2);
  if (!signed_channel_speeds) {
    RCLCPP_ERROR(this->get_logger(), "Rejected command because motor signs are invalid");
    return;
  }
  serial_worker_->submitCommand(
    signed_channel_speeds->channel_1_mps,
    signed_channel_speeds->channel_2_mps);
  diagnostics_loop();
}

void Roboteq::odom_setup()
{
  RCLCPP_INFO(this->get_logger(), "setting up odom...");
  if (pub_odom_tf) {
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(
      this->get_logger(),
      "Dynamic odom TF enabled: %s -> %s",
      odom_frame.c_str(),
      base_frame.c_str());
  }

  // maybe use this-> instead of


  odom_msg.header.stamp = this->get_clock()->now();

  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;

  const auto covariance_config =
    sanitize_covariance_config_with_logging(this->get_logger(), odom_covariance_config_);
  odom_msg.pose.covariance =
    roboteq_ros2_driver::odom_covariance::build_pose_covariance(covariance_config);
  odom_msg.twist.covariance =
    roboteq_ros2_driver::odom_covariance::build_twist_covariance(covariance_config);

  // start encoder streaming
  RCLCPP_INFO_STREAM(this->get_logger(), "covariance set");
  RCLCPP_INFO_STREAM(this->get_logger(), "odometry polling will be handled by serial worker");
}

// Odom msg streams


void Roboteq::odom_loop()
{
  if (!serial_worker_) {
    return;
  }

  const auto sample = serial_worker_->takeLatestEncoderSample();
  if (!sample.has_value() || !sample->valid) {
    return;
  }


  const auto now = std::chrono::steady_clock::now();
  double dt = 0.0;
  if (odom_last_time_valid_) {
    const auto elapsed = roboteq_ros2_driver::odometry::monotonic_elapsed_interval(
      odom_last_time, now);
    if (!elapsed.has_value()) {
      return;
    }
    dt = *elapsed;
  } else {
    odom_last_time_valid_ = true;
  }
  odom_last_time = now;

  // RCLCPP_INFO(this->get_logger(), "Odom Delta Time: %f", dt);

  const auto integration = odometry_integrator_.integrate_channel_sample(
    sample->channel_1,
    sample->channel_2,
    dt,
    channel_1,
    channel_2,
    encoder_sign_1,
    encoder_sign_2);
  if (!integration.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Invalid channel configuration");
    return;
  }

  publish_ticks(integration->ticks.left_ticks, integration->ticks.right_ticks);
  odom_publish(*integration);

  return;     // early return if no encoders read
}


void Roboteq::publish_ticks(int left_ticks, int right_ticks)
{
  roboteq_ros2_driver::msg::WheelTicks msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "ticks_frame";
  msg.left_ticks = left_ticks;
  msg.right_ticks = right_ticks;

  msg.right_ticks_norm =
    static_cast<double>(right_ticks) / encoder_cpr;      // convert ticks to turns
  msg.left_ticks_norm = static_cast<double>(left_ticks) / encoder_cpr;    // convert ticks to turns

  ticks_publisher_->publish(msg);
}


void Roboteq::odom_publish(const roboteq_ros2_driver::odometry::IntegrationResult & integration)
{
  odom_x = integration.pose.x;
  odom_y = integration.pose.y;
  odom_yaw = integration.pose.theta;

  // convert yaw to quat;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, odom_yaw);
  // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2_quat);

  // odom_msg.header.seq++; //? not used in ros2 ?
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = integration.twist.linear_x;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = integration.twist.angular_z;
  if (odom_tf_broadcaster_) {
    odom_tf_broadcaster_->sendTransform(
      roboteq_ros2_driver::odom_tf::build_odom_to_base_transform(
        odom_frame,
        base_frame,
        odom_msg.header.stamp,
        odom_x,
        odom_y,
        odom_yaw));
  }
  odom_pub->publish(odom_msg);
  // odom_pub.publish(odom_msg); ROS1
}

void Roboteq::command_watchdog_loop()
{
  bool should_log_timeout = false;
  {
    std::lock_guard<std::mutex> lock(command_state_mutex_);
    const double command_age_s = received_first_cmd_ ?
      (this->now() - last_cmd_time_).seconds() : 0.0;
    should_log_timeout = roboteq_ros2_driver::command_watchdog::should_send_timeout_stop(
      received_first_cmd_, command_timeout_logged_, command_age_s, cmd_timeout_s_);
    if (should_log_timeout) {
      command_timeout_logged_ = true;
    }
  }
  if (should_log_timeout) {
    RCLCPP_WARN(this->get_logger(), "cmd_vel timeout; serial worker will enforce stop");
  }
  diagnostics_loop();
}

void Roboteq::diagnostics_loop()
{
  if (!diagnostics_pub_ || !serial_worker_) {
    return;
  }
  std::lock_guard<std::mutex> publication_lock(diagnostics_publication_mutex_);

  const auto worker_status = serial_worker_->status();
  roboteq_ros2_driver::DiagnosticsState state;
  state.serial_connected = worker_status.transport_open;
  state.serial_ready = worker_status.ready_for_motion;
  state.encoder_sample_available = worker_status.have_encoder_sample;
  state.worker_status = worker_status;
  if (worker_status.have_encoder_sample) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - worker_status.latest_encoder_timestamp);
    state.encoder_age = age;
  }
  {
    std::lock_guard<std::mutex> lock(command_state_mutex_);
    state.command_active = received_first_cmd_;
    if (received_first_cmd_) {
      const auto command_age_s = (this->now() - last_cmd_time_).seconds();
      state.command_timed_out = command_age_s >= cmd_timeout_s_;
      state.command_age = std::chrono::milliseconds(
        static_cast<int>(command_age_s * 1000.0));
    }
  }

  roboteq_ros2_driver::DiagnosticsConfig config;
  config.publish_period = std::chrono::milliseconds(
    static_cast<int>(std::max(1.0, 1000.0 / std::max(0.001, diagnostics_publish_rate_hz_))));
  config.encoder_freshness_warn = std::chrono::milliseconds(
    static_cast<int>(encoder_freshness_warn_s_ * 1000.0));
  config.encoder_freshness_error = std::chrono::milliseconds(
    static_cast<int>(encoder_freshness_error_s_ * 1000.0));
  config.command_watchdog_error = std::chrono::milliseconds(
    static_cast<int>(std::max(1.0, cmd_timeout_s_ * 1000.0)));
  config.command_watchdog_warn = std::chrono::milliseconds(
    static_cast<int>(std::max<int64_t>(1, config.command_watchdog_error.count() / 2)));

  const auto msg = roboteq_ros2_driver::buildDiagnosticsArray(this->now(), state, config);
  roboteq_ros2_driver::DiagnosticsPublicationDecision decision;
  decision = diagnostics_state_.evaluate(
    msg,
    std::chrono::steady_clock::now(),
    config.publish_period);
  if (!decision.publish) {
    return;
  }
  diagnostics_pub_->publish(msg);
  log_diagnostics_records(
    this->get_logger(),
    roboteq_ros2_driver::buildDiagnosticsLogRecords(msg));
}

Roboteq::~Roboteq()
{
  if (serial_worker_) {
    serial_worker_->stop();
  }
  // rclcpp::shutdown(); // uncomment if node doesnt destroy properly
}

void Roboteq::start_serial_worker()
{
  roboteq_ros2_driver::SerialTransportConfig transport_config;
  transport_config.port = port;
  transport_config.baud = baud;
  transport_config.read_timeout = std::chrono::milliseconds(serial_read_timeout_ms_);
  transport_config.write_timeout = std::chrono::milliseconds(serial_write_timeout_ms_);
  transport_config.transaction_timeout =
    std::chrono::milliseconds(serial_transaction_timeout_ms_);
  transport_config.max_response_bytes = static_cast<std::size_t>(serial_max_response_bytes_);

  roboteq_ros2_driver::SerialWorkerConfig worker_config;
  worker_config.open_loop = open_loop;
  worker_config.wheel_circumference = wheel_circumference;
  worker_config.max_rpm = max_rpm;
  worker_config.command_timeout = std::chrono::milliseconds(
    static_cast<int>(std::max(0.001, cmd_timeout_s_) * 1000.0));
  worker_config.encoder_poll_period = std::chrono::milliseconds(encoder_poll_period_ms_);
  worker_config.reconnect_interval = std::chrono::milliseconds(
    static_cast<int>(serial_reconnect_interval_s_ * 1000.0));
  worker_config.require_fresh_command_after_reconnect = require_fresh_command_after_reconnect_;
  worker_config.required_settings =
    roboteq_ros2_driver::configuration::required_controller_settings(
    open_loop, encoder_eppr, max_amps, max_rpm);
  worker_config.log_callback = [logger = this->get_logger()](const std::string & message) {
      RCLCPP_WARN(logger, "%s", message.c_str());
    };
  serial_worker_ = std::make_unique<roboteq_ros2_driver::SerialIoWorker>(
    std::make_unique<roboteq_ros2_driver::RoboteqSerialTransport>(transport_config),
    worker_config);
  serial_worker_->start();
}

}  // namespace Roboteq

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Roboteq::Roboteq>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
