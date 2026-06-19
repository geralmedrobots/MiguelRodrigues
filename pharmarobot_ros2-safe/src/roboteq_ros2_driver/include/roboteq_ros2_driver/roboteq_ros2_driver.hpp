#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_

#include <math.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <cmath>
#include <climits>

#include "differential_drive_kinematics.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include "roboteq_ros2_driver/msg/wheel_ticks.hpp"
#include "roboteq_ros2_driver/odom_covariance.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"


#define PI M_PI 
namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
  public:
  explicit Roboteq(); //(nodeOptions options?)
  ~Roboteq();

  private:

  rclcpp::Time last_cmd_time_;
  bool received_first_cmd_ = false;
  bool command_timeout_logged_ = false;
  bool odom_twist_initialized_ = false;
  double cmd_timeout_s_ = 0.5;

  DifferentialDriveKinematics differential_drive_kinematics_;

  //sDifferentialDriveKinematics differential_drive_kinematics_;
  // class atributes
  //rclcpp::Node::SharedPtr nh{};
  RobotPose current_pose{0.0, 0.0, 0.0}; // Initialize current pose with zero values
  uint32_t starttime{};
  uint32_t hstimer{};
  uint32_t mstimer{};
  uint32_t lstimer{};
  rclcpp::TimerBase::SharedPtr command_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  std::unique_ptr<roboteq_ros2_driver::SerialIoWorker> serial_worker_;

  // buffer for reading encoder counts
  unsigned int odom_idx{};
  char odom_buf[24]{};

  // toss out initial encoder readings
  char odom_encoder_toss{};

  int32_t odom_encoder_left{};
  int32_t odom_encoder_right{};

  //std::optional<EncoderToAngularVelocityConverter> encoder_converter_;
  
  //DifferentialDriveKinematics kinematics;
  //EncoderToAngularVelocityConverter encoder_to_angular_velocity;

  int32_t ch1_odom_encoder{};
  int32_t ch2_odom_encoder{};


  float odom_x{};
  float odom_y{};
  float odom_yaw{};
  float odom_last_x{};
  float odom_last_y{};
  float odom_last_yaw{};

  uint32_t odom_last_time{};


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
  roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig odom_covariance_config_{};
  // Test different odom msg memory
  //nav_msgs::msg::Odometry odom_msg{};
  nav_msgs::msg::Odometry odom_msg{};
  //geometry_msgs::msg::Twist twist_msg{};



    //
  // cmd_vel subscriber
  //
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();

  //
  // odom publisher
  //
  void odom_setup();
  void odom_loop();
  //void odom_hs_run();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish(int left_ticks, int right_ticks, double dt);
  void publish_ticks(int left_ticks,int right_ticks);

  void update_parameters();
  void command_watchdog_loop();
  void start_serial_worker();

  //subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

  //publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<roboteq_ros2_driver::msg::WheelTicks>::SharedPtr ticks_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;


};

}





#endif
