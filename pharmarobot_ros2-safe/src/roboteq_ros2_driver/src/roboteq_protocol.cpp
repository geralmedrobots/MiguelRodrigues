#include "roboteq_ros2_driver/roboteq_protocol.hpp"

#include <charconv>
#include <cstddef>
#include <string_view>
#include <system_error>

namespace roboteq_ros2_driver
{
namespace protocol
{
namespace
{

std::string_view trim_line_endings(const std::string & response)
{
  std::string_view view(response);
  while (!view.empty() && (view.back() == '\r' || view.back() == '\n')) {
    view.remove_suffix(1);
  }
  return view;
}

bool consume_prefix(std::string_view * view, std::string_view prefix)
{
  if (view->substr(0, prefix.size()) != prefix) {
    return false;
  }
  view->remove_prefix(prefix.size());
  return true;
}

std::optional<int> parse_int(std::string_view text)
{
  if (text.empty()) {
    return std::nullopt;
  }

  int value = 0;
  const char * first = text.data();
  const char * last = text.data() + text.size();
  const auto result = std::from_chars(first, last, value);
  if (result.ec != std::errc{} || result.ptr != last) {
    return std::nullopt;
  }
  return value;
}

std::optional<std::vector<int>> parse_colon_separated_ints(
  std::string_view view, std::size_t expected_count)
{
  if (view.empty()) {
    return std::nullopt;
  }

  std::vector<int> values;
  while (true) {
    const std::size_t separator = view.find(':');
    const std::string_view field = view.substr(0, separator);
    const std::optional<int> value = parse_int(field);
    if (!value.has_value()) {
      return std::nullopt;
    }
    values.push_back(*value);

    if (separator == std::string_view::npos) {
      break;
    }
    view.remove_prefix(separator + 1);
  }

  if (values.size() != expected_count) {
    return std::nullopt;
  }
  return values;
}

}  // namespace

std::optional<std::string> parse_firmware_id(const std::string & response)
{
  std::string_view view = trim_line_endings(response);
  if (!consume_prefix(&view, "FID=") || view.empty()) {
    return std::nullopt;
  }
  return std::string(view);
}

std::optional<std::vector<int>> parse_voltage_fields(const std::string & response)
{
  std::string_view view = trim_line_endings(response);
  if (!consume_prefix(&view, "V=")) {
    return std::nullopt;
  }
  return parse_colon_separated_ints(view, 3);
}

std::optional<std::pair<int, int>> parse_encoder_counts(const std::string & response)
{
  std::string_view view = trim_line_endings(response);
  if (!consume_prefix(&view, "CR=")) {
    return std::nullopt;
  }

  std::optional<std::vector<int>> values = parse_colon_separated_ints(view, 2);
  if (!values.has_value()) {
    return std::nullopt;
  }
  return std::make_pair((*values)[0], (*values)[1]);
}

}  // namespace protocol
}  // namespace roboteq_ros2_driver
