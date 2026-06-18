# roboteq_ros2_driver



ROS2 driver for the Roboteq SDC21xx, HDC24xx family of motor controllers in a differential-drive configuration.
Initially developed for SDC21xx and HDC24xx, but could work with other roboteq dual-channel motor drivers.

Subscribes to cmd_vel, publishes to odom


Does not require any MicroBasic script to operate.

## Usage

Clone to src directory of ros2 workspace, then `colcon build` 

Requires serial package, which is not available as deb in ROS2. If not already installed, install ros2 branch of serial:

Get the code:
    
    git clone -b ros2 https://github.com/SunnyApp-Robotics/serial.git

Build:

    make

Install:

    make install
    
    
Sample launch files in roboteq_ros2_driver/launch, or run `ros2 run roboteq_ros2_driver roboteq_ros2_driver`

## Encoder response parsing safety

Encoder count responses from the Roboteq controller are parsed as strict
`CR=<int>:<int>` protocol messages. The runtime driver uses the same parser
covered by `test_roboteq_protocol`.

Malformed encoder responses are rejected instead of being partially accepted.
For example, partial numeric fields such as `CR=12x:34` or `CR=12:34y`,
empty fields, unexpected prefixes, and out-of-range integers are invalid.
Invalid encoder responses follow the existing safe invalid-sentinel path in
`readEncoderCountRelative()` and return `INT_MAX` for both channels.

This parser robustness change is limited to rejecting malformed encoder input.
It does not intentionally change motor-command generation, serial write
commands, launch files, hardware parameters, or safety logic.

## Parser-only validation

After building the workspace, run the Roboteq protocol parser test with:

    colcon test --packages-select roboteq_ros2_driver --ctest-args -R test_roboteq_protocol

Inspect results with:

    colcon test-result --verbose

The validated parser test result was:

    Summary: 8 tests, 0 errors, 0 failures, 0 skipped

## ROS Foxy build note

This package is built in ROS Foxy using `rosidl_target_interfaces(...)` for the
generated `WheelTicks` message typesupport. The driver header also supports the
Foxy `tf2_geometry_msgs/tf2_geometry_msgs.h` include path while remaining
compatible with newer `tf2_geometry_msgs/tf2_geometry_msgs.hpp` installations.

Known non-blocking build output: the driver may report an unrelated unused
`dt` variable warning in `driver_dev.cpp`.

## Motor Power Connections

This driver assumes right motor is connected to channel 1 (M1) of motor controller, and left motor is connected to channel 2 (M2). It also assumes a positive speed command will result in forward motion of each motor. Best to test motor directions using the roboteq utility software.


## TODO

- [X] Initial ROS2 release with motor commands and odometry stream
- [ ] Implement transform broadcasting with tf2
- [ ] Add roboteq/voltage, roboteq/current, roboteq/energy, and roboteq/temperature publishers
- [ ] Make topic names and frames configuration parameters configurable at runtime.
- [ ] Make robot configuration parameters configurable at runtime.
- [ ] Make motor controller device configuration parameters configurable at runtime.
- [ ] Make miscellaneous motor controller configuration parameters configurable at runtime.
- [ ] Implement dynamically enabled self-test mode to verify correct motor power and encoder connections and configuration.

### Note: I do not have access to Roboteq hardware anymore - feel free to contribute!
[original work for ROS1](https://github.com/ecostech/roboteq_diff_driver)
## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)
* **Chase Devitt** - *ROS2 Migration*
