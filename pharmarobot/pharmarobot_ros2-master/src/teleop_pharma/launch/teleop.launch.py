import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    roboteq_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roboteq_ros2_driver'),
                'launch',
                'roboteq_ros2_driver.launch.py'
            )
        )
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen'
    )

    joy_to_cmdvel_node = Node(
        package='joy_to_cmdvel',
        executable='joy_to_cmd_vel_node',
        name='joy_to_cmd_vel_node',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        joy_to_cmdvel_node,
        roboteq_launch,
    ])
