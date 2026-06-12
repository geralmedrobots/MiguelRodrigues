from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    roboteq_config = str(
        Path(get_package_share_directory("roboteq_ros2_driver"))
        / "config"
        / "roboteq.yaml"
    )

    joy = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.08,
            "autorepeat_rate": 20.0,
        }],
    )

    joy_to_cmdvel = Node(
        package="joy_to_cmdvel",
        executable="joy_to_cmd_vel_node",
        name="joy_to_cmdvel",
        output="screen",
        parameters=[{
            "joy_timeout_s": 0.5,
            "enable_button_index": 4,
                "reverse_steering": True,  # L1 on the current PlayStation mapping
            "output_topic": "/cmd_vel/joy",
        }],
    )

    command_arbiter = Node(
        package="command_arbiter",
        executable="command_arbiter_node",
        name="command_arbiter",
        output="screen",
        parameters=[{
            "output_topic": "/cmd_vel/safe",
            "publish_rate_hz": 20.0,
            "max_linear_x": 1.0,
            "max_angular_z": 0.9,
            "joy_topic": "/cmd_vel/joy",
            "joy_priority": 100,
            "joy_timeout_s": 0.25,
            "test_topic": "/cmd_vel/test",
            "test_priority": 50,
            "test_timeout_s": 0.25,
            "navigation_topic": "/cmd_vel/nav",
            "navigation_priority": 10,
            "navigation_timeout_s": 0.25,
        }],
    )

    roboteq = Node(
        package="roboteq_ros2_driver",
        executable="roboteq_ros2_driver_node",
        name="roboteq_ros2_driver",
        output="screen",
        parameters=[roboteq_config],
    )

    return LaunchDescription([joy, joy_to_cmdvel, command_arbiter, roboteq])
