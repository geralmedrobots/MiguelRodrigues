#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr int LINEAR_AXIS_INDEX = 1;
constexpr int ANGULAR_AXIS_INDEX = 3;
constexpr int DEFAULT_L1_BUTTON_INDEX = 4;
constexpr int R2_BUTTON_INDEX = 7;
constexpr int R2_AXIS_INDEX = 5;
constexpr double R2_AXIS_PRESSED_THRESHOLD = -0.5;

constexpr double LINEAR_AXIS_SIGN = 1.0;
constexpr double ANGULAR_AXIS_SIGN = 1.0;

constexpr double NORMAL_MAX_LINEAR_VEL = 0.6;
constexpr double NORMAL_MAX_ANGULAR_VEL = 0.5;
constexpr double TURBO_MAX_LINEAR_VEL = 1.0;
constexpr double TURBO_MAX_ANGULAR_VEL = 0.9;
constexpr double JOYSTICK_DEADZONE = 0.08;
constexpr double LINEAR_ACCEL_LIMIT = 1.2;
constexpr double ANGULAR_ACCEL_LIMIT = 1.8;

using namespace std::chrono_literals;

double clamp_value(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

double apply_deadzone_and_scale(double raw_value)
{
    const double value = clamp_value(raw_value, -1.0, 1.0);

    if (std::abs(value) < JOYSTICK_DEADZONE) {
        return 0.0;
    }

    const double sign = value >= 0.0 ? 1.0 : -1.0;
    const double magnitude =
        (std::abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE);

    return sign * clamp_value(magnitude, 0.0, 1.0);
}

double ramp_towards(double current, double target, double max_delta)
{
    if (target > current + max_delta) {
        return current + max_delta;
    }

    if (target < current - max_delta) {
        return current - max_delta;
    }

    return target;
}

bool is_r2_pressed(const sensor_msgs::msg::Joy::SharedPtr & msg)
{
    bool r2_button_pressed = false;
    bool r2_axis_pressed = false;

    if (msg->buttons.size() > static_cast<size_t>(R2_BUTTON_INDEX)) {
        r2_button_pressed = msg->buttons[R2_BUTTON_INDEX] != 0;
    }

    if (msg->axes.size() > static_cast<size_t>(R2_AXIS_INDEX)) {
        r2_axis_pressed = msg->axes[R2_AXIS_INDEX] < R2_AXIS_PRESSED_THRESHOLD;
    }

    return r2_button_pressed || r2_axis_pressed;
}
}  // namespace

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmdvel")
    {
        joy_timeout_s_ = this->declare_parameter<double>("joy_timeout_s", 0.5);
        enable_button_index_ = this->declare_parameter<int>(
            "enable_button_index", DEFAULT_L1_BUTTON_INDEX);
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel/joy");
        reverse_steering_ = this->declare_parameter<bool>(
            "reverse_steering", true);

        if (enable_button_index_ < 0) {
            throw std::invalid_argument("enable_button_index must be zero or greater");
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::SensorDataQoS(),
            std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
        watchdog_timer_ = this->create_wall_timer(100ms, std::bind(&JoyToCmdVel::watchdog_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "JoyToCmdVel started: output=%s linear axis=%d angular axis=%d "
            "L1 enable button index=%d joy timeout=%.2f s",
            output_topic_.c_str(),
            LINEAR_AXIS_INDEX,
            ANGULAR_AXIS_INDEX,
            enable_button_index_,
            joy_timeout_s_
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    bool has_last_output_time_ = false;
    bool received_joy_ = false;
    bool timeout_stop_sent_ = false;
    bool deadman_stop_sent_ = false;

    rclcpp::Time last_output_time_;
    rclcpp::Time last_joy_time_;

    double last_linear_x_ = 0.0;
    double last_angular_z_ = 0.0;
    double joy_timeout_s_ = 0.5;
    int enable_button_index_ = DEFAULT_L1_BUTTON_INDEX;
    std::string output_topic_;
    bool reverse_steering_ = true;

    void publish_stop(const char * reason)
    {
        geometry_msgs::msg::Twist stop_msg;
        publisher_->publish(stop_msg);
        last_linear_x_ = 0.0;
        last_angular_z_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "Published stop command: %s", reason);
    }

    void publish_deadman_stop_once(const char * reason)
    {
        if (!deadman_stop_sent_ || last_linear_x_ != 0.0 || last_angular_z_ != 0.0) {
            publish_stop(reason);
        }

        deadman_stop_sent_ = true;
        has_last_output_time_ = false;
    }

    void watchdog_callback()
    {
        if (!received_joy_ || timeout_stop_sent_) {
            return;
        }

        const double age_s = (this->now() - last_joy_time_).seconds();
        if (std::isfinite(age_s) && age_s > joy_timeout_s_) {
            publish_stop("joystick message timeout");
            timeout_stop_sent_ = true;
            deadman_stop_sent_ = true;
            has_last_output_time_ = false;
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_time_ = this->now();
        received_joy_ = true;
        timeout_stop_sent_ = false;

        const size_t required_axis_count =
            static_cast<size_t>(std::max(LINEAR_AXIS_INDEX, ANGULAR_AXIS_INDEX) + 1);

        if (msg->axes.size() < required_axis_count) {
            publish_deadman_stop_once("Joy message does not contain the required axes");
            return;
        }

        const size_t enable_index = static_cast<size_t>(enable_button_index_);
        if (msg->buttons.size() <= enable_index) {
            publish_deadman_stop_once("Joy message does not contain the configured L1 enable button");
            return;
        }

        const bool enable_active = msg->buttons[enable_index] != 0;
        if (!enable_active) {
            publish_deadman_stop_once("L1 hold-to-run button released");
            return;
        }

        deadman_stop_sent_ = false;

        const bool turbo_active = is_r2_pressed(msg);
        const double max_linear_vel =
            turbo_active ? TURBO_MAX_LINEAR_VEL : NORMAL_MAX_LINEAR_VEL;
        const double max_angular_vel =
            turbo_active ? TURBO_MAX_ANGULAR_VEL : NORMAL_MAX_ANGULAR_VEL;

        const double normalized_linear = apply_deadzone_and_scale(msg->axes[LINEAR_AXIS_INDEX]);
        const double normalized_angular = apply_deadzone_and_scale(msg->axes[ANGULAR_AXIS_INDEX]);

        const double target_linear_x =
            LINEAR_AXIS_SIGN * normalized_linear * max_linear_vel;

        // ROS angular.z represents yaw direction independently of linear motion.
        // For car-like joystick behaviour, reverse the steering command while
        // travelling backwards. Pure rotation remains unchanged because
        // normalized_linear is zero.
        const double reverse_steering_sign =
            reverse_steering_ && normalized_linear < 0.0 ? -1.0 : 1.0;

        const double target_angular_z =
            ANGULAR_AXIS_SIGN *
            normalized_angular *
            max_angular_vel *
            reverse_steering_sign;

        const auto now = this->now();
        double dt = 0.02;

        if (has_last_output_time_) {
            dt = (now - last_output_time_).seconds();
            if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
                dt = 0.02;
            }
        } else {
            has_last_output_time_ = true;
        }

        last_output_time_ = now;

        const double linear_x = ramp_towards(
            last_linear_x_, target_linear_x, LINEAR_ACCEL_LIMIT * dt);
        const double angular_z = ramp_towards(
            last_angular_z_, target_angular_z, ANGULAR_ACCEL_LIMIT * dt);

        last_linear_x_ = linear_x;
        last_angular_z_ = angular_z;

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        publisher_->publish(twist_msg);

        RCLCPP_DEBUG(
            this->get_logger(),
            "JOY_TO_CMDVEL mode=%s target_linear=%.3f target_angular=%.3f linear=%.3f angular=%.3f",
            turbo_active ? "TURBO" : "NORMAL",
            target_linear_x,
            target_angular_z,
            linear_x,
            angular_z
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
