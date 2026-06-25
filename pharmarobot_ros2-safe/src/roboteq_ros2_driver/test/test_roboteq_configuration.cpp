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

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace configuration = roboteq_ros2_driver::configuration;

namespace
{

const configuration::RequiredControllerSetting * findSetting(
  const std::vector<configuration::RequiredControllerSetting> & settings,
  const std::string & name,
  int channel)
{
  const auto it = std::find_if(
    settings.begin(),
    settings.end(),
    [&name, channel](const auto & setting) {
      return setting.name == name && setting.channel == channel;
    });
  return it == settings.end() ? nullptr : &*it;
}

}  // namespace

TEST(RoboteqConfiguration, BuildsClosedLoopRequiredSettings)
{
  const auto settings = configuration::required_controller_settings(false, -1024, 5.0, 100);

  ASSERT_EQ(settings.size(), 20u);
  ASSERT_NE(findSetting(settings, "MMOD", 1), nullptr);
  EXPECT_EQ(findSetting(settings, "MMOD", 1)->expected_value, 1);
  EXPECT_EQ(findSetting(settings, "MMOD", 2)->expected_value, 1);
  EXPECT_EQ(findSetting(settings, "ALIM", 1)->expected_value, 50);
  EXPECT_EQ(findSetting(settings, "MXRPM", 2)->expected_value, 100);
  EXPECT_EQ(findSetting(settings, "EPPR", 1)->expected_value, -1024);
  EXPECT_EQ(findSetting(settings, "ECHOF", 0)->expected_value, 1);
  EXPECT_EQ(findSetting(settings, "RWD", 0)->expected_value, 1000);
}

TEST(RoboteqConfiguration, BuildsOpenLoopMotorMode)
{
  const auto settings = configuration::required_controller_settings(true, -2048, 4.5, 80);

  EXPECT_EQ(findSetting(settings, "MMOD", 1)->expected_value, 0);
  EXPECT_EQ(findSetting(settings, "MMOD", 2)->expected_value, 0);
  EXPECT_EQ(findSetting(settings, "ALIM", 1)->expected_value, 45);
  EXPECT_EQ(findSetting(settings, "MXRPM", 1)->expected_value, 80);
  EXPECT_EQ(findSetting(settings, "EPPR", 2)->expected_value, -2048);
}
