#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"


#include <chrono> 
#include <functional> 
#include <memory>     
#include <string>     

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>

#include "std_msgs/msg/string.hpp"

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


#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>


serial::Serial controller;
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
    cmdvel_topic = this->declare_parameter("cmdvel_topic", "/cmd_vel");
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

    RCLCPP_INFO(this->get_logger(), "Parameters initialized ...");
    this->differential_drive_kinematics_->initParam(wheel_radius, wheelbase, encoder_cpr);

    
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
    cmdvel_setup();
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


void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) // const???
{
    
    // wheel speed (m/s)
    float right_speed = twist_msg->linear.x + wheelbase * twist_msg->angular.z / 2.0;
    float left_speed = twist_msg->linear.x - wheelbase * twist_msg->angular.z / 2.0;

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

    if (open_loop)
    {
        // motor power (scale 0-1000)
        RCLCPP_INFO_STREAM(this->get_logger(),"open loop");
        int32_t channel_1_power = channel_1_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
        int32_t channel_2_power = channel_2_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;

        
        channel_1_cmd << "!G 1 " << channel_1_power << "\r";
        channel_2_cmd << "!G 2 " << channel_2_power << "\r";
    }
    else
    {
        // motor speed (rpm)
        int32_t channel_1_rpm = channel_1_speed / wheel_circumference * 60.0;
        int32_t channel_2_rpm = channel_2_speed / wheel_circumference * 60.0;
        
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
    controller.write(channel_1_cmd.str());
    controller.write(channel_2_cmd.str());
    controller.flush();
    //#endif
}

void Roboteq::cmdvel_setup()
{
    RCLCPP_INFO(this->get_logger(), "configuring motor controller...");

    // stop motors
    controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();

    // disable echo
    controller.write("^ECHOF 1\r");
    controller.flush();

    // enable watchdog timer (1000 ms)
    controller.write("^RWD 1000\r");

    

    // set motor operating mode (1 for closed-loop speed)
    if (open_loop)
    {
        // open-loop speed mode
        controller.write("^MMOD 1 0\r");
        controller.write("^MMOD 2 0\r");
    }
    else
    {
        // closed-loop speed mode
        controller.write("^MMOD 1 1\r");
        controller.write("^MMOD 2 1\r");
    }

    
    // set motor amps limit (A * 10)
    std::stringstream right_ampcmd;
    std::stringstream left_ampcmd;
    right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
    left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
    controller.write(right_ampcmd.str());
    controller.write(left_ampcmd.str());

    // set max speed (rpm) for relative speed commands
    std::stringstream right_rpmcmd;
    std::stringstream left_rpmcmd;
    right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
    left_rpmcmd  << "^MXRPM 2 " << max_rpm << "\r";
    
    controller.write(right_rpmcmd.str());
    controller.write(left_rpmcmd.str());

    // set max acceleration rate (2000 rpm/s * 10)
    controller.write("^MAC 1 20000\r");
    controller.write("^MAC 2 20000\r");

    // set max deceleration rate (2000 rpm/s * 10)
    controller.write("^MDEC 1 20000\r");
    controller.write("^MDEC 2 20000\r");

    
    
    
    // set PID parameters (gain * 10)
    controller.write("^KP 1 1\r");
    controller.write("^KP 2 1\r");
    controller.write("^KI 1 7\r");
    controller.write("^KI 2 7\r");
    controller.write("^KD 1 0\r");
    controller.write("^KD 2 0\r");
    
    
    // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
    //controller.write("^EMOD 1 18\r");
    //controller.write("^EMOD 2 34\r");
    
    
    // set encoder counts (ppr)
    std::stringstream right_enccmd;
    std::stringstream left_enccmd;

    right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
    left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
    
    controller.write(right_enccmd.str());
    controller.write(left_enccmd.str());

    controller.flush();
}

void Roboteq::cmdvel_loop()
{
}


void Roboteq::odom_setup()
{
    RCLCPP_INFO(this->get_logger(),"setting up odom...");
    if (pub_odom_tf)
    {
        //TODO: implement tf2 broadcaster
        
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

    // Set up the transform message: move to odom_publish
    /*
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw);

    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    */

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
    RCLCPP_INFO_STREAM(this->get_logger(),"SEND to DRIVER: # C_?CR_# 33\r");

#endif
    controller.flush();
}

std::vector<int> Roboteq::readEncoderCountRelative()
{
    // Send encoder query to the controller
    controller.flushInput();
    controller.write("?CR\r");
    controller.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

    // Parse result, expected format: "CR=1234:5678"
    if (!result.empty() && result.find("CR=") == 0)
    {
        size_t colon_pos = result.find(':');
        if (colon_pos != std::string::npos)
        {
            int ch1 = std::stoi(result.substr(3, colon_pos - 3));
            int ch2 = std::stoi(result.substr(colon_pos + 1));
            output.push_back(ch1);
            output.push_back(ch2);
            //RCLCPP_WARN(this->get_logger(), "Found encoder counts: %d, %d", ch1, ch2);
        }
        else
        {
            output.push_back(INT_MAX);
            output.push_back(INT_MAX);
        }
    }
    else
    {
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

    RobotDisplacement twist = differential_drive_kinematics_->calculateForwardKinematics(right_ticks, left_ticks);

    current_pose = differential_drive_kinematics_->updateRobotPose(current_pose, twist);


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



    //tf2::Quaternion quat = tf2::createQuaternionMsgFromYaw(odom_yaw);
    // TODO: set up tf2_ros
    //update odom msg

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
    odom_pub->publish(odom_msg);
    // odom_pub.publish(odom_msg); ROS1
}

int Roboteq::run()
{

    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;
    
    //cmdvel_loop();
    odom_loop();
    //cmdvel_run();
    return 0;
}

Roboteq::~Roboteq()
{

    if (controller.isOpen()){
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
