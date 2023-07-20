#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
import compas_fab
from compas.robots import Configuration
from compas_fab.backends import PyBulletClient
import pathlib
import time
from compas.geometry import Frame



class EEPose(Node):

    def __init__(self):
        super().__init__('ee_pose')
        self.publisher_ = self.create_publisher(String, 'ee_pose', 10)
        self.joint_publisher_ = self.create_publisher(Float64MultiArray,'/forward_position_controller/commands',10)
        self.subscriber_ = self.create_subscription(
            String,
            'ee_pose',
            self.listener_callback,
            10)
        msg = String()
        x = 0.6
        y = -0.2
        z = 0.525
        msg.data = str(x) + ' ' + str(y) + ' ' + str(z)
        self.orientation_ = [[[1,0,0],[0,1,0]],[[0,0,-1],[0,1,0]],[[1,0,0],[0,0,1]]]
        self.i = 0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        print("IN HERE")
        time.sleep(1)
        xyz_str = msg.data
        xyz_arr = xyz_str.split(' ')
        x = float(xyz_arr[0])
        y = float(xyz_arr[1])
        z = float(xyz_arr[2])
        with PyBulletClient() as client:
            urdf_filename = compas_fab.get('../../../../../../cross_embodiment_test_ws/install/gazebo_env/share/gazebo_env/urdf/ur5e_analytical.urdf')
            robot = client.load_robot(urdf_filename)
            print([x,y,z])
            print(self.orientation_[self.i][0])
            print(self.orientation_[self.i][1])
            frame_WCF = Frame([x, y, z], self.orientation_[self.i][0], self.orientation_[self.i][1])
            self.i += 1
            start_configuration = robot.zero_configuration()

            configuration = robot.inverse_kinematics(frame_WCF, start_configuration)
            joint_message = Float64MultiArray()
            joint_message.data = configuration.joint_values
            
            print("Found configuration", configuration)
            self.joint_publisher_.publish(joint_message)
            print("PRIME STUFF")

def main(args=None):
    rclpy.init(args=args)

    ee_pose = EEPose()

    rclpy.spin(ee_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ee_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()