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

#include "roboteq_ros2_driver/roboteq_configuration.hpp"

#include <vector>

namespace roboteq_ros2_driver
{
namespace configuration
{

std::vector<RequiredControllerSetting> required_controller_settings(
  bool open_loop,
  int encoder_eppr,
  double max_amps,
  int max_rpm)
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
    {"EPPR", 1, encoder_eppr},
    {"EPPR", 2, encoder_eppr},
  };
}

}  // namespace configuration
}  // namespace roboteq_ros2_driver
