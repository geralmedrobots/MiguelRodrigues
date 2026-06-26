// Copyright 2026 Medrobots
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "roboteq_ros2_driver/roboteq_protocol.hpp"

#include <charconv>
#include <string_view>
#include <cstddef>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

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

std::optional<int> parse_config_readback(
  const std::string & response, const std::string & setting_name)
{
  std::string_view view = trim_line_endings(response);
  if (setting_name.empty()) {
    return std::nullopt;
  }

  const std::string prefix = setting_name + "=";
  if (!consume_prefix(&view, prefix)) {
    return std::nullopt;
  }
  return parse_int(view);
}

}  // namespace protocol
}  // namespace roboteq_ros2_driver
