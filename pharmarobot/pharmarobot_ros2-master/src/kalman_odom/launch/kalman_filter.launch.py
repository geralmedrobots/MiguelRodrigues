# kalman_filter.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalman_odom',
            executable='kalman_odometry_node',
            name='kalman_odometry',
            parameters=[{
                'initial_r': 0.05,
                'initial_b': 0.3,
                'Q_diag': [0.001, 0.001, 0.001, 0.001, 0.001],
                'R_diag': [0.5, 0.5, 0.05],
            }],
            output='screen'
        ),

        Node(
            package='kalman_odom',
            executable='test_sim_input.py',
            name='sim_test_input',
            output='screen'
        )
    ])
