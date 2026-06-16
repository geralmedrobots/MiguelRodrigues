#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;

struct SourceState
{
  SourceState(
    std::string source_name,
    std::string source_topic,
    int source_priority,
    double source_timeout_s,
    rcl_clock_type_t clock_type)
  : name(std::move(source_name)),
    topic(std::move(source_topic)),
    priority(source_priority),
    timeout_s(source_timeout_s),
    last_received(0, 0, clock_type)
  {
  }

  std::string name;
  std::string topic;
  int priority;
  double timeout_s;
  geometry_msgs::msg::Twist last_command;
  rclcpp::Time last_received;
  bool has_command{false};
};

bool command_is_finite(const geometry_msgs::msg::Twist & command)
{
  return std::isfinite(command.linear.x) && std::isfinite(command.angular.z);
}

double clamp_symmetric(double value, double absolute_limit)
{
  return std::clamp(value, -absolute_limit, absolute_limit);
}
}  // namespace

class CommandArbiter final : public rclcpp::Node
{
public:
  CommandArbiter()
  : Node("command_arbiter")
  {
    const std::string output_topic =
      this->declare_parameter<std::string>("output_topic", "/cmd_vel/safe");
    const double publish_rate_hz =
      this->declare_parameter<double>("publish_rate_hz", 20.0);
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 1.0);
    max_angular_z_ = this->declare_parameter<double>("max_angular_z", 0.9);

    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
    }
    if (!std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0) {
      throw std::invalid_argument("max_linear_x must be finite and greater than zero");
    }
    if (!std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0) {
      throw std::invalid_argument("max_angular_z must be finite and greater than zero");
    }

    const auto clock_type = this->get_clock()->get_clock_type();
    add_source(
      "joy",
      this->declare_parameter<std::string>("joy_topic", "/cmd_vel/joy"),
      this->declare_parameter<int>("joy_priority", 100),
      this->declare_parameter<double>("joy_timeout_s", 0.25),
      clock_type);
    add_source(
      "test",
      this->declare_parameter<std::string>("test_topic", "/cmd_vel/test"),
      this->declare_parameter<int>("test_priority", 50),
      this->declare_parameter<double>("test_timeout_s", 0.25),
      clock_type);
    add_source(
      "navigation",
      this->declare_parameter<std::string>("navigation_topic", "/cmd_vel/nav"),
      this->declare_parameter<int>("navigation_priority", 10),
      this->declare_parameter<double>("navigation_timeout_s", 0.25),
      clock_type);

    output_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(1).reliable());

    subscriptions_.reserve(sources_.size());
    for (std::size_t index = 0; index < sources_.size(); ++index) {
      subscriptions_.push_back(
        this->create_subscription<geometry_msgs::msg::Twist>(
          sources_[index].topic,
          rclcpp::QoS(1).reliable(),
          [this, index](const geometry_msgs::msg::Twist::SharedPtr message) {
            source_callback(index, *message);
          }));
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CommandArbiter::publish_selected_command, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Command arbiter started: output=%s, sources=%zu, rate=%.1f Hz",
      output_topic.c_str(),
      sources_.size(),
      publish_rate_hz);

    for (const auto & source : sources_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Source '%s': topic=%s priority=%d timeout=%.3f s",
        source.name.c_str(),
        source.topic.c_str(),
        source.priority,
        source.timeout_s);
    }
  }

private:
  std::vector<SourceState> sources_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double max_linear_x_{1.0};
  double max_angular_z_{0.9};
  std::string active_source_{"none"};

  void add_source(
    const std::string & name,
    const std::string & topic,
    int priority,
    double timeout_s,
    rcl_clock_type_t clock_type)
  {
    if (topic.empty()) {
      throw std::invalid_argument("Command-source topic cannot be empty");
    }
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
      throw std::invalid_argument("Command-source timeout must be finite and greater than zero");
    }

    sources_.emplace_back(name, topic, priority, timeout_s, clock_type);
  }

  void source_callback(std::size_t index, const geometry_msgs::msg::Twist & message)
  {
    if (index >= sources_.size()) {
      return;
    }

    auto & source = sources_[index];

    if (!command_is_finite(message)) {
      source.has_command = false;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Rejected non-finite command from source '%s'",
        source.name.c_str());
      return;
    }

    geometry_msgs::msg::Twist sanitized;
    sanitized.linear.x = clamp_symmetric(message.linear.x, max_linear_x_);
    sanitized.angular.z = clamp_symmetric(message.angular.z, max_angular_z_);

    source.last_command = sanitized;
    source.last_received = this->now();
    source.has_command = true;
  }

  void publish_selected_command()
  {
    const auto now = this->now();
    const SourceState * selected_source = nullptr;

    for (const auto & source : sources_) {
      if (!source.has_command) {
        continue;
      }

      const double age_s = (now - source.last_received).seconds();
      if (!std::isfinite(age_s) || age_s < 0.0 || age_s > source.timeout_s) {
        continue;
      }

      if (
        selected_source == nullptr ||
        source.priority > selected_source->priority ||
        (source.priority == selected_source->priority &&
        source.last_received > selected_source->last_received))
      {
        selected_source = &source;
      }
    }

    geometry_msgs::msg::Twist output;
    const std::string selected_name = selected_source == nullptr ? "none" : selected_source->name;

    if (selected_source != nullptr) {
      output = selected_source->last_command;
    }

    output_publisher_->publish(output);

    if (selected_name != active_source_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Active command source changed: %s -> %s",
        active_source_.c_str(),
        selected_name.c_str());
      active_source_ = selected_name;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<CommandArbiter>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("command_arbiter"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
