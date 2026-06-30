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

#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "roboteq_ros2_driver/driver_parameter_validation.hpp"

namespace validation = roboteq_ros2_driver::parameter_validation;

namespace
{

validation::DriverParameters valid_parameters()
{
  return {
    "/dev/roboteq",
    115200,
    0.0881,
    0.453,
    1024,
    4096,
    -1024,
    1,
    1,
    1,
    1,
    1,
    5.0,
    100,
    0.5,
    50,
    50,
    500,
    256,
    1.0,
    50,
    1.0,
    "left",
    "right",
  };
}

void expect_invalid(
  const validation::DriverParameters & parameters,
  const std::string & expected_parameter)
{
  const auto error = validation::validate(parameters);
  ASSERT_TRUE(error.has_value());
  EXPECT_EQ(error->parameter, expected_parameter);
  EXPECT_FALSE(error->reason.empty());
}

}  // namespace

TEST(DriverParameterValidation, AcceptsProductionConfigurationAndBothExplicitSigns)
{
  auto parameters = valid_parameters();
  EXPECT_FALSE(validation::validate(parameters).has_value());

  parameters.motor_sign_1 = -1;
  parameters.motor_sign_2 = -1;
  parameters.encoder_sign_1 = -1;
  parameters.encoder_sign_2 = -1;
  parameters.command_angular_sign = 1;
  EXPECT_FALSE(validation::validate(parameters).has_value());
}

TEST(DriverParameterValidation, RejectsInvalidGeometry)
{
  for (const double value : {
    0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()
  })
  {
    auto parameters = valid_parameters();
    parameters.wheel_radius = value;
    expect_invalid(parameters, "wheel_radius");

    parameters = valid_parameters();
    parameters.wheelbase = value;
    expect_invalid(parameters, "wheelbase");
  }
}

TEST(DriverParameterValidation, RejectsZeroAndUnrepresentableEncoderMagnitudes)
{
  auto parameters = valid_parameters();
  parameters.encoder_ppr = 0;
  expect_invalid(parameters, "encoder_ppr");

  parameters = valid_parameters();
  parameters.encoder_cpr = 0;
  expect_invalid(parameters, "encoder_cpr");

  parameters = valid_parameters();
  parameters.encoder_ppr = std::numeric_limits<int>::min();
  expect_invalid(parameters, "encoder_ppr");

  parameters = valid_parameters();
  parameters.encoder_cpr = std::numeric_limits<int>::min();
  expect_invalid(parameters, "encoder_cpr");

  parameters = valid_parameters();
  parameters.encoder_eppr = 0;
  expect_invalid(parameters, "encoder_eppr");

  parameters = valid_parameters();
  parameters.encoder_eppr = std::numeric_limits<int>::min();
  expect_invalid(parameters, "encoder_eppr");
}

TEST(DriverParameterValidation, AcceptsPositiveEncoderMagnitudesAndSignedControllerEPPR)
{
  auto parameters = valid_parameters();
  parameters.encoder_eppr = -1024;
  EXPECT_FALSE(validation::validate(parameters).has_value());

  parameters = valid_parameters();
  parameters.encoder_ppr = 1024;
  parameters.encoder_cpr = 4096;
  EXPECT_FALSE(validation::validate(parameters).has_value());
}

TEST(DriverParameterValidation, RejectsInvalidExplicitSigns)
{
  for (const int value : {-2, 0, 2}) {
    auto parameters = valid_parameters();
    parameters.motor_sign_1 = value;
    expect_invalid(parameters, "motor_sign_1");

    parameters = valid_parameters();
    parameters.motor_sign_2 = value;
    expect_invalid(parameters, "motor_sign_2");

    parameters = valid_parameters();
    parameters.encoder_sign_1 = value;
    expect_invalid(parameters, "encoder_sign_1");

    parameters = valid_parameters();
    parameters.encoder_sign_2 = value;
    expect_invalid(parameters, "encoder_sign_2");

    parameters = valid_parameters();
    parameters.command_angular_sign = value;
    expect_invalid(parameters, "command_angular_sign");
  }
}

TEST(DriverParameterValidation, RejectsInvalidRpmAndCurrentLimits)
{
  for (const int value : {-1, 0}) {
    auto parameters = valid_parameters();
    parameters.max_rpm = value;
    expect_invalid(parameters, "max_rpm");
  }

  for (const double value : {
    -1.0, 0.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()
  })
  {
    auto parameters = valid_parameters();
    parameters.max_amps = value;
    expect_invalid(parameters, "max_amps");
  }
}

TEST(DriverParameterValidation, EnforcesDocumentedSoftwareConversionUpperBounds)
{
  const double max_seconds =
    static_cast<double>(std::numeric_limits<int>::max()) / 1000.0;
  const double max_amps =
    static_cast<double>(std::numeric_limits<int>::max()) / 10.0;

  auto parameters = valid_parameters();
  parameters.command_timeout_s = max_seconds;
  parameters.serial_reconnect_interval_s = max_seconds;
  parameters.max_amps = max_amps;
  EXPECT_FALSE(validation::validate(parameters).has_value());

  parameters = valid_parameters();
  parameters.command_timeout_s = max_seconds + 1.0;
  expect_invalid(parameters, "cmd_timeout_s");

  parameters = valid_parameters();
  parameters.serial_reconnect_interval_s = max_seconds + 1.0;
  expect_invalid(parameters, "serial_reconnect_interval_s");

  parameters = valid_parameters();
  parameters.max_amps = max_amps + 1.0;
  expect_invalid(parameters, "max_amps");
}

TEST(DriverParameterValidation, RejectsInvalidDurations)
{
  for (const double value : {
    -1.0, 0.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()
  })
  {
    auto parameters = valid_parameters();
    parameters.command_timeout_s = value;
    expect_invalid(parameters, "cmd_timeout_s");

    parameters = valid_parameters();
    parameters.serial_reconnect_interval_s = value;
    expect_invalid(parameters, "serial_reconnect_interval_s");
  }

  for (const int value : {-1, 0}) {
    auto parameters = valid_parameters();
    parameters.serial_read_timeout_ms = value;
    expect_invalid(parameters, "serial_read_timeout_ms");

    parameters = valid_parameters();
    parameters.serial_write_timeout_ms = value;
    expect_invalid(parameters, "serial_write_timeout_ms");

    parameters = valid_parameters();
    parameters.serial_transaction_timeout_ms = value;
    expect_invalid(parameters, "serial_transaction_timeout_ms");

    parameters = valid_parameters();
    parameters.encoder_poll_period_ms = value;
    expect_invalid(parameters, "encoder_poll_period_ms");
  }
}

TEST(DriverParameterValidation, RejectsInvalidChannelMapping)
{
  auto parameters = valid_parameters();
  parameters.channel_1 = "motor_1";
  expect_invalid(parameters, "channel_1");

  parameters = valid_parameters();
  parameters.channel_2 = "motor_2";
  expect_invalid(parameters, "channel_2");

  parameters = valid_parameters();
  parameters.channel_2 = "left";
  expect_invalid(parameters, "channel_2");
}

TEST(DriverParameterValidation, RejectsInvalidSerialEndpointAndResponseSize)
{
  auto parameters = valid_parameters();
  parameters.port.clear();
  expect_invalid(parameters, "port");

  parameters = valid_parameters();
  parameters.baud = 0;
  expect_invalid(parameters, "baud");

  parameters = valid_parameters();
  parameters.baud = -1;
  expect_invalid(parameters, "baud");

  parameters = valid_parameters();
  parameters.serial_max_response_bytes = 0;
  expect_invalid(parameters, "serial_max_response_bytes");

  parameters = valid_parameters();
  parameters.serial_max_response_bytes = -1;
  expect_invalid(parameters, "serial_max_response_bytes");
}

TEST(DriverParameterValidation, InvalidStartupCannotCreateOrOpenTransportOrCreateWorker)
{
  auto parameters = valid_parameters();
  parameters.wheel_radius = 0.0;
  int transport_constructions = 0;
  int transport_opens = 0;
  int worker_constructions = 0;
  int ros_entity_constructions = 0;
  int controller_or_motion_commands = 0;

  const auto error = validation::validate_then_start(
    parameters, [&]() {
      ++ros_entity_constructions;
      ++transport_constructions;
      ++transport_opens;
      ++worker_constructions;
      ++controller_or_motion_commands;
    });

  ASSERT_TRUE(error.has_value());
  EXPECT_EQ(error->parameter, "wheel_radius");
  EXPECT_EQ(transport_constructions, 0);
  EXPECT_EQ(transport_opens, 0);
  EXPECT_EQ(worker_constructions, 0);
  EXPECT_EQ(ros_entity_constructions, 0);
  EXPECT_EQ(controller_or_motion_commands, 0);
}

TEST(DriverParameterValidation, ValidStartupCrossesSerialStartupBoundaryOnce)
{
  int startup_calls = 0;
  const auto error = validation::validate_then_start(
    valid_parameters(), [&startup_calls]() {++startup_calls;});
  EXPECT_FALSE(error.has_value());
  EXPECT_EQ(startup_calls, 1);
}
