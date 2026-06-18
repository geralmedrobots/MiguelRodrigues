#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_PROTOCOL_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_PROTOCOL_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace roboteq_ros2_driver
{
namespace protocol
{

std::optional<std::string> parse_firmware_id(const std::string & response);
std::optional<std::vector<int>> parse_voltage_fields(const std::string & response);
std::optional<std::pair<int, int>> parse_encoder_counts(const std::string & response);

}  // namespace protocol
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_PROTOCOL_HPP_
