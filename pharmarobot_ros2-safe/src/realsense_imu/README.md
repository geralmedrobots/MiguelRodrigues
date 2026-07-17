# RealSense D455 IMU

This package starts the official `realsense2_camera` ROS 2 wrapper with only
the D455 gyro and accelerometer enabled. Color, depth, infrared, RGBD, point
cloud, alignment, and RealSense TF publishing are disabled.

The upstream wrapper combines the two motion streams using linear
interpolation and supplies the sample timestamp. `realsense_imu` relays that
message to a configurable topic and frame without changing angular velocity,
linear acceleration, timestamp, or their covariances. Orientation is not
estimated: its covariance element zero is `-1`, as required by
`sensor_msgs/msg/Imu` for unavailable orientation.
The launch path automatically adds the underscore prefix required internally
by `realsense2_camera` for digit-only serial numbers.

The launch defaults are:

- serial number: `151223061922`;
- output topic: `/camera/imu` (`sensor_msgs/msg/Imu`);
- frame ID: `d455_gyro_optical_frame`;
- angular-velocity covariance: `0.01` on the upstream wrapper;
- linear-acceleration covariance: `0.01` on the upstream wrapper.

The `frame_id` override changes only the message label. It must name a frame
with the same axes as the RealSense gyro optical frame; this package does not
publish TF or transform IMU vectors.

## Dependencies

Install the ROS 2 Humble RealSense wrapper and SDK packages from the configured
ROS repository:

```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera \
  ros-humble-realsense2-description ros-humble-librealsense2
```

For `rs-enumerate-devices`, also install the RealSense SDK utilities from the
official RealSense package repository (`librealsense2-utils`). The launch file
fails with an explicit dependency error if `realsense2_camera` or its launch
file is unavailable. Device/SDK startup errors are reported by the official
wrapper, with a five-second startup device timeout.

## Hardware preflight

These checks inspect the camera but do not start ROS streams:

```bash
lsusb -d 8086:0b5c
rs-enumerate-devices -s
```

Confirm the product is `Intel(R) RealSense(TM) Depth Camera 455` and the serial
is `151223061922`. If ROS runs in a container, that container must separately
be granted access to the camera's `/dev/bus/usb` device; this package does not
broaden the control container's USB permissions.

## Run

Hardware execution requires separate approval under this repository's safety
rules. Once approved and the camera is accessible:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense_imu
source install/setup.bash
ros2 launch realsense_imu d455_imu.launch.py \
  serial_number:=151223061922 \
  frame_id:=d455_gyro_optical_frame \
  topic_name:=/camera/imu
```

Inspect one message and confirm that no image or depth topics are present:

```bash
ros2 topic echo /camera/imu sensor_msgs/msg/Imu --once
ros2 topic list
```
