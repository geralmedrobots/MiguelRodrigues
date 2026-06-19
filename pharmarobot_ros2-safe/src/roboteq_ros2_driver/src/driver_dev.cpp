#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"
#include "roboteq_ros2_driver/command_watchdog.hpp"
#include "roboteq_ros2_driver/command_scaling.hpp"
#include "roboteq_ros2_driver/odom_covariance.hpp"
#include "roboteq_ros2_driver/odom_tf.hpp"
#include "roboteq_ros2_driver/odom_twist.hpp"
#include "roboteq_ros2_driver/roboteq_protocol.hpp"
#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"


#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional> 
#include <memory>     
#include <optional>
#include <stdexcept>
#include <string>     
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>

// dependencies for ROS
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))

#define _CMDVEL_DEBUG

//#define _VERBOSE


// Define following to enable odom debug output
#define _ODOM_DEBUG



#define NORMALIZE(_z) atan2(sin(_z), cos(_z))


#define ROBOTEQ_CYCLE_PERIOD 50ms // ms
#define ROBORTEQ_WRITING_TIMEOUT 5 //


#include <tf2/LinearMath/Quaternion.h>



namespace
{
constexpr double kCommandAngularSign = -1.0;

std::vector<roboteq_ros2_driver::RequiredControllerSetting> required_controller_settings(
    bool open_loop, int encoder_ppr, double max_amps, int max_rpm)
{
    const int motor_mode = open_loop ? 0 : 1;
    const int amp_limit = static_cast<int>(max_amps * 10);

    return {
        {"ECHOF", 0, 1},
        {"RWD", 0, 1000},
        {"MMOD", 1, motor_mode},
        {"MMOD", 2, motor_mode},
        {"ALIM", 1, amp_limit},
        {"ALIM", 2, amp_limit},
        {"MXRPM", 1, max_rpm},
        {"MXRPM", 2, max_rpm},
        {"MAC", 1, 20000},
        {"MAC", 2, 20000},
        {"MDEC", 1, 20000},
        {"MDEC", 2, 20000},
        {"KP", 1, 1},
        {"KP", 2, 1},
        {"KI", 1, 7},
        {"KI", 2, 7},
        {"KD", 1, 0},
        {"KD", 2, 0},
        {"EPPR", 1, encoder_ppr},
        {"EPPR", 2, encoder_ppr},
    };
}

int sanitize_positive_int_parameter(
    const rclcpp::Logger & logger,
    const char * name,
    int value,
    int fallback)
{
    if (value > 0) {
        return value;
    }
    RCLCPP_WARN(logger, "Invalid parameter '%s'=%d; using default %d", name, value, fallback);
    return fallback;
}

double sanitize_positive_double_parameter(
    const rclcpp::Logger & logger,
    const char * name,
    double value,
    double fallback)
{
    if (std::isfinite(value) && value > 0.0) {
        return value;
    }
    RCLCPP_WARN(logger, "Invalid parameter '%s'=%.6f; using default %.6f", name, value, fallback);
    return fallback;
}

double sanitize_covariance_parameter(
    const rclcpp::Logger & logger,
    const char * name,
    double value,
    double fallback)
{
    const double sanitized = roboteq_ros2_driver::odom_covariance::sanitize_variance(value, fallback);
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

roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig sanitize_covariance_config_with_logging(
    const rclcpp::Logger & logger,
    const roboteq_ros2_driver::odom_covariance::OdometryCovarianceConfig & config)
{
    const auto defaults = roboteq_ros2_driver::odom_covariance::default_config();
    return {
        sanitize_covariance_parameter(logger, "odom_pose_covariance_x", config.pose_x, defaults.pose_x),
        sanitize_covariance_parameter(logger, "odom_pose_covariance_y", config.pose_y, defaults.pose_y),
        sanitize_covariance_parameter(logger, "odom_pose_covariance_z", config.pose_z, defaults.pose_z),
        sanitize_covariance_parameter(logger, "odom_pose_covariance_roll", config.pose_roll, defaults.pose_roll),
        sanitize_covariance_parameter(logger, "odom_pose_covariance_pitch", config.pose_pitch, defaults.pose_pitch),
        sanitize_covariance_parameter(logger, "odom_pose_covariance_yaw", config.pose_yaw, defaults.pose_yaw),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_linear_x", config.twist_linear_x, defaults.twist_linear_x),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_linear_y", config.twist_linear_y, defaults.twist_linear_y),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_linear_z", config.twist_linear_z, defaults.twist_linear_z),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_angular_x", config.twist_angular_x, defaults.twist_angular_x),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_angular_y", config.twist_angular_y, defaults.twist_angular_y),
        sanitize_covariance_parameter(
            logger, "odom_twist_covariance_angular_z", config.twist_angular_z, defaults.twist_angular_z),
    };
}
}

uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
Roboteq::Roboteq() : Node("roboteq_ros2_driver")
//differential_drive_kinematics_(void)
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
    wheel_radius = this->declare_parameter("wheel_radius", 0.085); // in meters
    wheelbase = this->declare_parameter("wheelbase", 0.453); // in meters
    encoder_ppr = this->declare_parameter("encoder_ppr", -1024);
    encoder_cpr = this->declare_parameter("encoder_cpr", -4096);
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
    odom_covariance_config_.twist_linear_x =
        this->declare_parameter("odom_twist_covariance_linear_x", default_covariance.twist_linear_x);
    odom_covariance_config_.twist_linear_y =
        this->declare_parameter("odom_twist_covariance_linear_y", default_covariance.twist_linear_y);
    odom_covariance_config_.twist_linear_z =
        this->declare_parameter("odom_twist_covariance_linear_z", default_covariance.twist_linear_z);
    odom_covariance_config_.twist_angular_x =
        this->declare_parameter("odom_twist_covariance_angular_x", default_covariance.twist_angular_x);
    odom_covariance_config_.twist_angular_y =
        this->declare_parameter("odom_twist_covariance_angular_y", default_covariance.twist_angular_y);
    odom_covariance_config_.twist_angular_z =
        this->declare_parameter("odom_twist_covariance_angular_z", default_covariance.twist_angular_z);
    serial_read_timeout_ms_ = this->declare_parameter("serial_read_timeout_ms", 50);
    serial_write_timeout_ms_ = this->declare_parameter("serial_write_timeout_ms", 50);
    serial_transaction_timeout_ms_ = this->declare_parameter("serial_transaction_timeout_ms", 100);
    serial_max_response_bytes_ = this->declare_parameter("serial_max_response_bytes", 256);
    serial_reconnect_interval_s_ = this->declare_parameter("serial_reconnect_interval_s", 1.0);
    encoder_poll_period_ms_ = this->declare_parameter("encoder_poll_period_ms", 50);
    require_fresh_command_after_reconnect_ =
        this->declare_parameter("require_fresh_command_after_reconnect", true);

    RCLCPP_INFO(this->get_logger(), "Parameters initialized ...");
    differential_drive_kinematics_.initParam(wheel_radius, wheelbase, encoder_cpr);

    
    starttime = 0;
    hstimer   = 0;
    mstimer   = 0;
    odom_idx  = 0;
    odom_encoder_toss  = 5;
    odom_encoder_left  = 0;
    odom_encoder_right = 0;
    ch1_odom_encoder   = 0;
    ch2_odom_encoder   = 0;
    odom_x         = 0.0;
    odom_y         = 0.0;
    odom_yaw       = 0.0;
    odom_last_x    = 0.0;
    odom_last_y    = 0.0;
    odom_last_yaw  = 0.0;
    odom_last_time = 0;

    wheel_circumference = 2*PI*wheel_radius;
    

    odom_msg = nav_msgs::msg::Odometry();

    update_parameters();
    odom_setup();
//
//  odom publisher
//
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 100);
    ticks_publisher_ = this->create_publisher<roboteq_ros2_driver::msg::WheelTicks>("wheel_ticks", 100);

//
// cmd_vel subscriber
//

    command_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    feedback_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cmdvel_options;
    cmdvel_options.callback_group = command_callback_group_;
    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1,         // QoS history depth
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
    this->get_parameter("require_fresh_command_after_reconnect", require_fresh_command_after_reconnect_);

    serial_read_timeout_ms_ = sanitize_positive_int_parameter(
        this->get_logger(), "serial_read_timeout_ms", serial_read_timeout_ms_, 50);
    serial_write_timeout_ms_ = sanitize_positive_int_parameter(
        this->get_logger(), "serial_write_timeout_ms", serial_write_timeout_ms_, 50);
    serial_transaction_timeout_ms_ = sanitize_positive_int_parameter(
        this->get_logger(), "serial_transaction_timeout_ms", serial_transaction_timeout_ms_, 100);
    serial_max_response_bytes_ = sanitize_positive_int_parameter(
        this->get_logger(), "serial_max_response_bytes", serial_max_response_bytes_, 256);
    serial_reconnect_interval_s_ = sanitize_positive_double_parameter(
        this->get_logger(), "serial_reconnect_interval_s", serial_reconnect_interval_s_, 1.0);
    encoder_poll_period_ms_ = sanitize_positive_int_parameter(
        this->get_logger(), "encoder_poll_period_ms", encoder_poll_period_ms_, 50);
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

    last_cmd_time_ = this->now();
    received_first_cmd_ = true;
    command_timeout_logged_ = false;

    // wheel speed (m/s)
    // ROS convention: positive angular.z = left turn.
    // Hardware-specific correction: this robot's physical turn direction is inverted,
    // so we invert only the angular component here.
    const double linear_x = twist_msg->linear.x;
    const double angular_z = kCommandAngularSign * twist_msg->angular.z;

    const double left_speed = linear_x - (wheelbase * angular_z / 2.0);

    const double right_speed = linear_x + (wheelbase * angular_z / 2.0);

    RCLCPP_DEBUG(
        this->get_logger(),
        "CMD_VEL_TO_WHEELS | linear_x=%.3f angular_z=%.3f | left_speed=%.3f right_speed=%.3f",
        twist_msg->linear.x,
        twist_msg->angular.z,
        left_speed,
        right_speed
    );

    //RCLCPP_INFO(this->get_logger(), "Received linear = %0.2f, angular = %0.2f", twist_msg->linear.x, twist_msg->angular.z);

    float channel_1_speed;
    float channel_2_speed;

    /**************** CHANNEL SWAP ***********************************/

    if(channel_1 == "right" && channel_2 == "left")
    { // Default
    
        channel_1_speed = right_speed;
        channel_2_speed = left_speed;
    }
    else if (channel_1 == "left" && channel_2 == "right")
    {
        channel_1_speed = left_speed;
        channel_2_speed = right_speed;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid channel configuration");
        return;
    }
    /**************** CHANNEL SWAP ***********************************/


    RCLCPP_DEBUG(
        this->get_logger(),
        "WHEELS_TO_CHANNELS | left_speed=%.3f right_speed=%.3f | channel_1=%s %.3f | channel_2=%s %.3f",
        left_speed,
        right_speed,
        channel_1.c_str(),
        channel_1_speed,
        channel_2.c_str(),
        channel_2_speed
    );

    serial_worker_->submitCommand(channel_1_speed, channel_2_speed);
}

void Roboteq::cmdvel_loop()
{
}


void Roboteq::odom_setup()
{
    RCLCPP_INFO(this->get_logger(),"setting up odom...");
    if (pub_odom_tf)
    {
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
    RCLCPP_INFO_STREAM(this->get_logger(),"covariance set");
    RCLCPP_INFO_STREAM(this->get_logger(),"odometry polling will be handled by serial worker");
    
    odom_last_time = millis();
#ifdef _ODOM_SENSORS
    current_last_time = millis();
#endif
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

    
    //uint32_t nowtime = millis();
    
    
    
    uint32_t nowtime = millis();
    double dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
    odom_last_time = nowtime;
    
    //RCLCPP_INFO(this->get_logger(), "Odom Delta Time: %f", dt);
    
    // encoders[0] = right, encoders[1] = left (or vice versa, depending on your config)
    // Use encoders for odometry update, e.g.:
    // ch1_odom_encoder = encoders[0];
    // if we haven't received encoder counts in some time then restart streaming
    ch1_odom_encoder =  sample->channel_1;
    ch2_odom_encoder =  sample->channel_2;

    if (ch1_odom_encoder == INT_MAX || ch2_odom_encoder == INT_MAX)
    {
        return; // early return if no encoders read
    }

    ch2_odom_encoder *=-1;
    ch1_odom_encoder *=-1;

        // *******************************************************
    if (channel_1 == "right" && channel_2 == "left")
    {
        odom_encoder_right = ch1_odom_encoder;
        odom_encoder_left  = ch2_odom_encoder;
    }
    else if (channel_1 == "left" && channel_2 == "right")
    {
        odom_encoder_right = ch2_odom_encoder;
        odom_encoder_left  = ch1_odom_encoder;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid channel configuration");
        return;
    }

    publish_ticks(odom_encoder_left, odom_encoder_right);
    odom_publish(odom_encoder_left, odom_encoder_right, dt);
    
    return ; // early return if no encoders read
}



void Roboteq::publish_ticks(int left_ticks,int right_ticks)
{

    roboteq_ros2_driver::msg::WheelTicks msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "ticks_frame";
    msg.left_ticks = left_ticks;
    msg.right_ticks = right_ticks;

    msg.right_ticks_norm =(-1)*(double)right_ticks / encoder_cpr; // convert ticks to radians
    msg.left_ticks_norm = (-1)*(double)left_ticks / encoder_cpr; // convert ticks to radians

    ticks_publisher_->publish(msg);
}



void Roboteq::odom_publish(int left_ticks, int right_ticks, double dt)
{

    RobotDisplacement displacement = differential_drive_kinematics_.calculateForwardKinematics(left_ticks, right_ticks);
    const double previous_yaw = current_pose.theta;

    current_pose = differential_drive_kinematics_.updateRobotPose(current_pose, displacement);


    odom_x = current_pose.x;
    odom_y = current_pose.y;
    odom_yaw = current_pose.theta;

    odom_last_x = odom_x;
    odom_last_y = odom_y;
    odom_last_yaw = odom_yaw;
    // convert yaw to quat;
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, odom_yaw);
    // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2_quat);

    //odom_msg.header.seq++; //? not used in ros2 ?
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;
    const auto measured_twist = roboteq_ros2_driver::odom_twist::calculate_measured_twist(
        displacement.linear_x,
        previous_yaw,
        odom_yaw,
        dt,
        odom_twist_initialized_);
    odom_msg.twist.twist.linear.x = measured_twist.linear_x;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = measured_twist.angular_z;
    odom_twist_initialized_ = true;
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
    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;

    const double command_age_s = received_first_cmd_ ?
        (this->now() - last_cmd_time_).seconds() : 0.0;
    if (roboteq_ros2_driver::command_watchdog::should_send_timeout_stop(
        received_first_cmd_, command_timeout_logged_, command_age_s, cmd_timeout_s_))
    {
        command_timeout_logged_ = true;
        RCLCPP_WARN(this->get_logger(), "cmd_vel timeout; serial worker will enforce stop");
    }
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
    transport_config.transaction_timeout = std::chrono::milliseconds(serial_transaction_timeout_ms_);
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
        required_controller_settings(open_loop, encoder_ppr, max_amps, max_rpm);
    worker_config.log_callback = [logger = this->get_logger()](const std::string & message) {
        RCLCPP_WARN(logger, "%s", message.c_str());
    };

    serial_worker_ = std::make_unique<roboteq_ros2_driver::SerialIoWorker>(
        std::make_unique<roboteq_ros2_driver::RoboteqSerialTransport>(transport_config),
        worker_config);
    serial_worker_->start();
}

} // end of namespace

int main(int argc, char* argv[])
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
