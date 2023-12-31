#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointCommandsPublisher(Node):
    def __init__(self):
        super().__init__('joint_commands_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_commands)

    def publish_joint_commands(self):
        # Creating a Float64MultiArray message
        msg = Float64MultiArray()
        
        # Example joint commands
        joint_commands = [-0.37793201, -1.51306807,  2.2465576 , -2.4317753 , -1.52886899,
       -1.80969565,0.0]  # Replace this with your joint commands
        
        # Assigning the joint commands to the Float64MultiArray message
        msg.data = joint_commands
        
        # Publishing the message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint commands: %s' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    joint_commands_publisher = JointCommandsPublisher()
    rclpy.spin(joint_commands_publisher)
    joint_commands_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
