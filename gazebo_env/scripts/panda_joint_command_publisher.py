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
        joint_commands = [-1.3841414917533297,  -1.7591862909653837, 1.7467437373749044, -2.738425771003514,
       1.7472542996335525,  1.4609213187098244, -0.15598075463265967,0.04]  # Replace this with your joint commands

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
