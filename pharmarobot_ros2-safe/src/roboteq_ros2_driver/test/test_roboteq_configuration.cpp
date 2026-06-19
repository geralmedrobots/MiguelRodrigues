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
