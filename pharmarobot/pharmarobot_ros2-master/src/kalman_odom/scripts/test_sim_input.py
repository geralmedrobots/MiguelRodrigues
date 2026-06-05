#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose2D
import math

class SimTestNode(Node):
    def __init__(self):
        super().__init__('sim_test_node')
        self.wheel_pub = self.create_publisher(Vector3, '/wheel_displacement', 10)
        self.lidar_pub = self.create_publisher(Pose2D, '/lidar/pose2d', 10)
        self.timer = self.create_timer(0.1, self.publish_mock_data)
        self.t = 0.0

    # straight line motion with slight oscillation
    # and occasional LIDAR correction
    # This simulates a robot moving forward with slight variations in wheel displacement
    # and provides LIDAR corrections at regular intervals.
    # The wheel displacements are simulated as small oscillations around a base value,
    # while the LIDAR corrections are provided at every 10th timer tick.
    
    def publish_straight_line_data(self):
        
        # generate random linear and angular velocities
        linear_velocity = 0.1 + 0.02 * math.sin(self.t)
        angular_velocity = 0.01 * math.cos(self.t)
        
        # Simulate wheel encoder displacement (in radians)
        dl = linear_velocity * 0.1 + 0.002 * math.sin(self.t)
        dr = linear_velocity * 0.1 - 0.002 * math.sin(self.t)   
        
    
    
    def publish_mock_data(self):
        # Simulate wheel encoder displacement (in radians)
        dl = 0.01 + 0.002 * math.sin(self.t)
        dr = 0.01 - 0.002 * math.sin(self.t)

        wheel_msg = Vector3()
        wheel_msg.x = dl
        wheel_msg.y = dr
        self.wheel_pub.publish(wheel_msg)

        # Simulate occasional LIDAR correction
        if int(self.t * 10) % 10 == 0:
            lidar_msg = Pose2D()
            lidar_msg.x = 0.1 * self.t
            lidar_msg.y = 0.05 * self.t
            lidar_msg.theta = 0.01 * self.t
            self.lidar_pub.publish(lidar_msg)

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = SimTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
