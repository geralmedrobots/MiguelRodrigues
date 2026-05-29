#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

constexpr double MAX_CMD_LINEAR_VEL = 0.4;
constexpr double MAX_CMD_ANG_VEL = 0.3;

double clip_max_cmd_linear_vel(double x) {
    return std::max(std::min(x, MAX_CMD_LINEAR_VEL), -MAX_CMD_LINEAR_VEL);
}

double clip_max_cmd_ang_vel(double x) {
    return std::max(std::min(x, MAX_CMD_ANG_VEL), -MAX_CMD_ANG_VEL);
}

double rescale_function(double value) {
    double scale = 0.1;
    if (std::abs(value) < 0.95) {
        value = value * scale;
    }
    return value;
}

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel() : Node("joy_to_cmdvel") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1,
            std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        RCLCPP_INFO(this->get_logger(), "RUNNING CPP JoyToCmdVel node");
        RCLCPP_INFO(this->get_logger(), "Using rescale function: rescale_function");
        RCLCPP_INFO(this->get_logger(), "JoyToCmdVel node has been started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto twist_msg = geometry_msgs::msg::Twist();

        // Adjust axes indices as needed for your joystick
        twist_msg.linear.x = clip_max_cmd_linear_vel(rescale_function(msg->axes[1]));
        twist_msg.linear.y = clip_max_cmd_linear_vel(rescale_function(msg->axes[0]));
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = -clip_max_cmd_ang_vel(rescale_function(msg->axes[3]));

        publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}