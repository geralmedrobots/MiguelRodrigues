#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cmath>

class SimTestNode : public rclcpp::Node {
public:
    SimTestNode() : Node("sim_test_node") {
        wheel_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/wheel_displacement", 10);
        lidar_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/lidar/pose2d", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimTestNode::publish_mock_data, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr lidar_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double t_ = 0.0;

    void publish_mock_data() {
        auto wheel_msg = geometry_msgs::msg::Vector3();
        wheel_msg.x = 0.01 + 0.002 * sin(t_);
        wheel_msg.y = 0.01 - 0.002 * sin(t_);
        wheel_pub_->publish(wheel_msg);

        if (static_cast<int>(t_ * 10) % 10 == 0) {  // simulate lidar at 1Hz
            auto lidar_msg = geometry_msgs::msg::Pose2D();
            lidar_msg.x = 0.1 * t_;
            lidar_msg.y = 0.05 * t_;
            lidar_msg.theta = 0.01 * t_;
            lidar_pub_->publish(lidar_msg);
        }

        t_ += 0.1;
    }
};
