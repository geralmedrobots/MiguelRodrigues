#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"
#include "roboteq_ros2_driver/odom_tf.hpp"
#include "roboteq_ros2_driver/roboteq_protocol.hpp"


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
#include <serial/serial.h>
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



serial::Serial controller;

namespace
{
constexpr double kCommandAngularSign = -1.0;
constexpr int kSerialReadDelayMs = 20;

struct RequiredConfigSetting
{
    const char * name;
    int channel;
    int expected_value;
};

std::vector<RequiredConfigSetting> required_controller_settings(
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

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);
    // connect to serial port
    
    update_parameters();
    // set up parameters for dynamic reconfigure
    connect();
    // configure motor controller
    try {
        cmdvel_setup();
    } catch (const std::exception & ex) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Roboteq startup validation failed; normal runtime startup aborted: %s",
            ex.what());
        if (controller.isOpen()) {
            send_stop_command("startup validation failed");
            controller.close();
        }
        throw;
    }
    odom_setup();
//
//  odom publisher
//
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 100);
    ticks_publisher_ = this->create_publisher<roboteq_ros2_driver::msg::WheelTicks>("wheel_ticks", 100);

//
// cmd_vel subscriber
//

    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1,         // QoS history depth
        std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));
    using namespace std::chrono_literals;
    // set odometry publishing loop timer at 10Hz
    timer_ = this->create_wall_timer(ROBOTEQ_CYCLE_PERIOD,std::bind(&Roboteq::run, this));
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

    

    
}

void Roboteq::connect(){
    RCLCPP_INFO_STREAM(this->get_logger(),"Opening serial port on " << port << " at " << baud << "..." );
    try
    {
        controller.open();
        if (controller.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
            return; 
            
        }
    }
    catch (serial::IOException &e)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "serial::IOException: ");
        throw;
    }
    RCLCPP_WARN(this->get_logger(),"Failed to open serial port");
    sleep(5);

}


void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    if (!controller_config_valid_) {
        RCLCPP_ERROR(this->get_logger(), "Rejected cmd_vel because Roboteq configuration is not validated");
        send_stop_command("controller configuration not validated");
        return;
    }

    if (!std::isfinite(twist_msg->linear.x) || !std::isfinite(twist_msg->angular.z)) {
        RCLCPP_ERROR(this->get_logger(), "Rejected non-finite cmd_vel command");
        send_stop_command("non-finite cmd_vel");
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

    std::stringstream channel_1_cmd;
    std::stringstream channel_2_cmd;
    
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

    if (open_loop)
    {
        // motor power (scale 0-1000)
        RCLCPP_INFO_STREAM(this->get_logger(),"open loop");
        int32_t channel_1_power = static_cast<int32_t>(
            channel_1_speed / wheel_circumference * 60.0 / max_rpm * 1000.0);
        int32_t channel_2_power = static_cast<int32_t>(
            channel_2_speed / wheel_circumference * 60.0 / max_rpm * 1000.0);
        channel_1_power = std::clamp(channel_1_power, -1000, 1000);
        channel_2_power = std::clamp(channel_2_power, -1000, 1000);

        
        channel_1_cmd << "!G 1 " << channel_1_power << "\r";
        channel_2_cmd << "!G 2 " << channel_2_power << "\r";
    }
    else
    {
        // motor speed (rpm)
        int32_t channel_1_rpm = static_cast<int32_t>(
            channel_1_speed / wheel_circumference * 60.0);
        int32_t channel_2_rpm = static_cast<int32_t>(
            channel_2_speed / wheel_circumference * 60.0);
        channel_1_rpm = std::clamp(channel_1_rpm, -max_rpm, max_rpm);
        channel_2_rpm = std::clamp(channel_2_rpm, -max_rpm, max_rpm);
        
        channel_1_cmd << "!S 1 " << channel_1_rpm << "\r";
        channel_2_cmd << "!S 2 " << channel_2_rpm << "\r";

    }

    #ifdef _VERBOSE
    printf("channel_1_cmd: %s\n", channel_1_cmd.str().c_str());
    printf("channel_2_cmd: %s\n\n", channel_2_cmd.str().c_str());
    #endif
    
    // send command to motor controller


    //write cmd to motor controller
    //#ifndef _CMDVEL_FORCE_RUN
    const std::string cmd_1 = channel_1_cmd.str();
    const std::string cmd_2 = channel_2_cmd.str();

    const size_t bytes_1 = controller.write(cmd_1);
    const size_t bytes_2 = controller.write(cmd_2);
    controller.flush();

    std::string cmd_1_print = cmd_1;
    std::string cmd_2_print = cmd_2;

    if (!cmd_1_print.empty() && cmd_1_print.back() == '\r') {
        cmd_1_print.pop_back();
    }

    if (!cmd_2_print.empty() && cmd_2_print.back() == '\r') {
        cmd_2_print.pop_back();
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "ROBOTEQ_SERIAL_TX | cmd1='%s' bytes1=%zu | cmd2='%s' bytes2=%zu",
        cmd_1_print.c_str(),
        bytes_1,
        cmd_2_print.c_str(),
        bytes_2
    );
    //#endif
}

void Roboteq::send_stop_command(const char * reason)
{
    if (!controller.isOpen()) {
        return;
    }

    try {
        controller.write("!G 1 0\r");
        controller.write("!G 2 0\r");
        controller.write("!S 1 0\r");
        controller.write("!S 2 0\r");
        controller.flush();
        RCLCPP_WARN(this->get_logger(), "Motor stop command sent: %s", reason);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send motor stop command: %s", ex.what());
    }
}

void Roboteq::cmdvel_setup()
{
    RCLCPP_INFO(this->get_logger(), "validating motor controller configuration...");
    controller_config_valid_ = false;

    // stop motors
    controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();

    if (!validate_controller_configuration()) {
        throw std::runtime_error("required Roboteq controller configuration does not match");
    }
    controller_config_valid_ = true;
    RCLCPP_INFO(this->get_logger(), "Roboteq controller configuration validated");
}

std::optional<int> Roboteq::read_controller_config_int(
    const std::string & setting_name, int channel)
{
    if (!controller.isOpen()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Cannot validate Roboteq setting %s: serial port is not open",
            setting_name.c_str());
        return std::nullopt;
    }

    std::stringstream query;
    query << "~" << setting_name;
    if (channel > 0) {
        query << " " << channel;
    }
    query << "\r";

    controller.flushInput();
    controller.write(query.str());
    controller.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(kSerialReadDelayMs));

    std::string line;
    while (controller.available())
    {
        char ch = 0;
        if (controller.read((uint8_t *)&ch, 1) == 0) {
            break;
        }

        if (ch == '\r' || ch == '\n') {
            if (!line.empty()) {
                const auto parsed = roboteq_ros2_driver::protocol::parse_config_readback(
                    line, setting_name);
                if (parsed.has_value()) {
                    return parsed;
                }
                line.clear();
            }
            continue;
        }
        line += ch;
    }

    if (!line.empty()) {
        const auto parsed = roboteq_ros2_driver::protocol::parse_config_readback(
            line, setting_name);
        if (parsed.has_value()) {
            return parsed;
        }
    }

    const std::string channel_label = channel > 0 ? " channel " + std::to_string(channel) : "";
    RCLCPP_ERROR(
        this->get_logger(),
        "Roboteq setting %s%s readback was missing or malformed",
        setting_name.c_str(),
        channel_label.c_str());
    return std::nullopt;
}

bool Roboteq::validate_controller_configuration()
{
    bool valid = true;
    for (const auto & setting : required_controller_settings(
        open_loop, encoder_ppr, max_amps, max_rpm))
    {
        const auto actual = read_controller_config_int(setting.name, setting.channel);
        if (!actual.has_value()) {
            valid = false;
            continue;
        }

        if (*actual != setting.expected_value) {
            const std::string channel_label =
                setting.channel > 0 ? " channel " + std::to_string(setting.channel) : "";
            RCLCPP_ERROR(
                this->get_logger(),
                "Roboteq configuration mismatch: %s%s expected %d but read %d",
                setting.name,
                channel_label.c_str(),
                setting.expected_value,
                *actual);
            valid = false;
        }
    }

    if (!valid) {
        send_stop_command("controller configuration validation failed");
    }
    return valid;
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

    // Set up the pose covariance
    for (size_t i = 0; i < 36; i++)
    {
        odom_msg.pose.covariance[i] = 0;
        odom_msg.twist.covariance[i] = 0;
    }

    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1000000;
    odom_msg.pose.covariance[21] = 1000000;
    odom_msg.pose.covariance[28] = 1000000;
    odom_msg.pose.covariance[35] = 1000;

    // Set up the twist covariance
    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[14] = 1000000;
    odom_msg.twist.covariance[21] = 1000000;
    odom_msg.twist.covariance[28] = 1000000;
    odom_msg.twist.covariance[35] = 1000;

    // start encoder streaming
    RCLCPP_INFO_STREAM(this->get_logger(),"covariance set");
    RCLCPP_INFO_STREAM(this->get_logger(),"odometry stream starting...");
    odom_stream();
    
    odom_last_time = millis();
#ifdef _ODOM_SENSORS
    current_last_time = millis();
#endif
}

// Odom msg streams


void Roboteq::odom_stream()
{

#ifdef _ODOM_SENSORS
    // start encoder and current output (30 hz)
    // doubling frequency since one value is output at each cycle
    //  controller.write("# C_?CR_?BA_# 17\r");
    // start encoder, current and voltage output (30 hz)
    // tripling frequency since one value is output at each cycle
    controller.write("# C_?CR_?BA_?V_# 11\r");
#else
    //  start encoder output (10 hz)
    //  controller.write("# C_?CR_# 100\r");
    // start encoder output (30 hz)
    //controller.write("# C_?CR_# 33\r");
    RCLCPP_INFO(this->get_logger(), "Encoder polling mode enabled");

#endif
    controller.flush();
}

std::vector<int> Roboteq::readEncoderCountRelative()
{
    // Send encoder query to the controller
    controller.flushInput();
    controller.write("?CR\r");
    controller.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(kSerialReadDelayMs));
    std::vector<int> output;
    std::string result;




    char ch = 0;
    std::string buffer;
    while (controller.available())
    {
        if (controller.read((uint8_t *)&ch, 1) == 0)
            break;
        if (ch == '\r')
            break;
        buffer += ch;
    }
    result = buffer;

    const auto encoder_counts = roboteq_ros2_driver::protocol::parse_encoder_counts(result);
    if (encoder_counts.has_value()) {
        output.push_back(encoder_counts->first);
        output.push_back(encoder_counts->second);
    } else {
        output.push_back(INT_MAX);
        output.push_back(INT_MAX);
    }
    return output;
}

void Roboteq::odom_loop()
{
    std::vector<int> encoders = readEncoderCountRelative();
    
    
    
    //uint32_t nowtime = millis();
    
    
    
    uint32_t nowtime = millis();
    double dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
    odom_last_time = nowtime;
    
    //RCLCPP_INFO(this->get_logger(), "Odom Delta Time: %f", dt);
    
    // encoders[0] = right, encoders[1] = left (or vice versa, depending on your config)
    // Use encoders for odometry update, e.g.:
    // ch1_odom_encoder = encoders[0];
    // if we haven't received encoder counts in some time then restart streaming
    ch1_odom_encoder =  encoders[0];
    ch2_odom_encoder =  encoders[1];

    if (ch1_odom_encoder == INT_MAX || ch2_odom_encoder == INT_MAX)
    {
        //RCLCPP_WARN(this->get_logger(), "No encoder data received, restarting odometry stream");
        //odom_stream();
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
    odom_publish(odom_encoder_left, odom_encoder_right);
    
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



void Roboteq::odom_publish(int left_ticks, int right_ticks)
{

    RobotDisplacement twist = differential_drive_kinematics_.calculateForwardKinematics(left_ticks, right_ticks);

    current_pose = differential_drive_kinematics_.updateRobotPose(current_pose, twist);


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
    odom_msg.twist.twist.linear.x = 0.0; // linear velocity in x
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
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

int Roboteq::run()
{
    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;

    if (received_first_cmd_) {
        const double command_age_s = (this->now() - last_cmd_time_).seconds();
        if (std::isfinite(command_age_s) && command_age_s > cmd_timeout_s_) {
            if (!command_timeout_logged_) {
                send_stop_command("cmd_vel timeout");
                command_timeout_logged_ = true;
            }
        }
    }

    odom_loop();
    return 0;
}

Roboteq::~Roboteq()
{
    if (controller.isOpen()) {
        send_stop_command("driver shutdown");
        controller.close();
    }
    // rclcpp::shutdown(); // uncomment if node doesnt destroy properly

}

} // end of namespace

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;

   
}
