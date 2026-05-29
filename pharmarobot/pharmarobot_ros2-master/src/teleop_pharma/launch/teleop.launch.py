from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

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

    return LaunchDescription([
        Node(
            package='joy_to_cmdvel',
            executable='joy_to_cmd_vel',
            name='joy_to_cmd_vel_node',
            output='screen'
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
            parameters=[{'deadzone': 0.001}]
        ),
        roboteq_launch,
    ])