from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('roboteq_ros2_driver'),
        'config',
        'roboteq.yaml'
    )

    roboteq_node = Node(
        package='roboteq_ros2_driver',
        executable='roboteq_ros2_driver_node',
        name='roboteq_ros2_driver_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        roboteq_node
    ])
