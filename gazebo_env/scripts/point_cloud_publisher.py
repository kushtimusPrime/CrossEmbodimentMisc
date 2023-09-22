#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import trimesh
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
import glob
import subprocess
import xml.etree.ElementTree as ET
from functools import partial
import math


class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.urdf_xacro_path_ = os.path.join(FindPackageShare(package="gazebo_env").find("gazebo_env"),"urdf","ur5e_gazebo_pointcloud_debug.urdf.xacro")
        xacro_command = "ros2 run xacro xacro " + self.urdf_xacro_path_
        xacro_subprocess = subprocess.Popen(
            xacro_command,
            shell=True,
            stdout=subprocess.PIPE,
        )
        urdf_string = ""
        while True:
            line = xacro_subprocess.stdout.readline()
            if line:
                line_byte = line.strip()
                line = line_byte.decode("utf-8")
                urdf_string += line
            else:
                break
        root = ET.fromstring(urdf_string)
        self.publishers_ = []
        self.timers_ = []
        timer_period = 0.5
        for link in root.iter('link'):
            element_name1 = "visual"
            found_element1 = link.find(".//" + element_name1)
            element_name2 = "geometry"
            found_element2 = link.find(".//" + element_name2)
            element_name3 = "mesh"
            found_element3 = link.find(".//" + element_name3)
            if (found_element1 is not None) and (found_element2 is not None) and (found_element3 is not None):
                link_name = link.attrib.get('name')
                for visual in link.iter("visual"):
                    origin_element = visual.find(".//origin")
                    rpy_str = origin_element.attrib.get('rpy')
                    xyz_str = origin_element.attrib.get('xyz')
                    for geometry in visual.iter("geometry"):
                        for mesh in geometry.iter("mesh"):
                            filename = mesh.attrib.get('filename')[7:]
                            publisher = self.create_publisher(PointCloud2,link_name+"_pointcloud",10)
                            self.publishers_.append(publisher)
                            timer = self.create_timer(timer_period,partial(self.debugTimerCallback,filename,link_name,publisher,rpy_str,xyz_str))
                            self.timers_.append(timer)

    def createTransform(self,rpy_str,xyz_str):
        rpy_array = rpy_str.split(' ')
        xyz_array = xyz_str.split(' ')
        roll = float(rpy_array[0])
        pitch = float(rpy_array[1])
        yaw = float(rpy_array[2])
        x = float(xyz_array[0])
        y = float(xyz_array[1])
        z = float(xyz_array[2])

        rotation_x = np.array([[1, 0, 0],
                           [0, math.cos(roll), -math.sin(roll)],
                           [0, math.sin(roll), math.cos(roll)]])

        rotation_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                            [0, 1, 0],
                            [-math.sin(pitch), 0, math.cos(pitch)]])

        rotation_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                            [math.sin(yaw), math.cos(yaw), 0],
                            [0, 0, 1]])

        rotation_matrix = np.dot(rotation_z, np.dot(rotation_y, rotation_x))
        translation_vector = np.array([[x],[y],[z]])
        transform_matrix = np.concatenate((rotation_matrix,translation_vector),axis=1)
        transform_matrix = np.concatenate((transform_matrix,np.array([[0,0,0,1]])),axis=0)
        return transform_matrix

    def debugTimerCallback(self,filename,link_name,publisher,rpy_str,xyz_str):
        mesh_scene = trimesh.load(filename)
        mesh = trimesh.util.concatenate(tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                                            for g in mesh_scene.geometry.values()))
        # Convert Trimesh to Open3D TriangleMesh
        vertices = o3d.utility.Vector3dVector(mesh.vertices)
        triangles = o3d.utility.Vector3iVector(mesh.faces)
        open3d_mesh = o3d.geometry.TriangleMesh(vertices, triangles)
        pcd = open3d_mesh.sample_points_uniformly(number_of_points=100000)
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) / 1000)
        pcd_data = np.asarray(pcd.points)
        pcd_data = pcd_data[:,[1,0,2]]
        pcd_data[:,0] *= -1
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = Header()
        point_cloud_msg.header.frame_id = link_name
        fields =[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(pcd_data)
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 3 * 4
        point_cloud_msg.row_step = point_cloud_msg.point_step * len(pcd_data)
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = bytearray(pcd_data.astype('float32').tobytes())

        publisher.publish(point_cloud_msg)

    def timerCallback(self,filename,link_name,publisher,rpy_str,xyz_str):

        mesh = trimesh.load(filename)
        transform_matrix = self.createTransform(rpy_str,xyz_str)
        o3d_mesh = None
        pcds = []
        points = None
        R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
        R2 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        for item in mesh.geometry:
            o3d_mesh = mesh.geometry[item].as_open3d
            #o3d_mesh = o3d_mesh.rotate(R)
            pcd = o3d_mesh.sample_points_uniformly(number_of_points=100000)
            if points is None:
                points = np.asarray(pcd.points)
            else:
                points = np.concatenate((points,pcd.points),axis=0)
            pcds.append(pcd)
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(points)
        center_translation = -new_pcd.get_center()
        new_pcd = new_pcd.translate(center_translation)
        new_pcd = new_pcd.rotate(R)
        new_pcd = new_pcd.rotate(R2)
        
        new_pcd = new_pcd.transform(transform_matrix)
        #print(transform_matrix)
        #mesh = mesh.apply_transform(transform_matrix)
        # vertices = None
        # for item in mesh.geometry:
        #     if vertices is None:
        #         vertices = mesh.geometry[item].vertices
        #     else:
        #         vertices = np.concatenate((vertices,mesh.geometry[item].vertices),axis=0)

        # # Create a visualization window
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(vertices)
        # R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
        # R2 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        # pcd = pcd.rotate(R)
        # pcd = pcd.rotate(R2)
        #pcd = pcd.transform(transform_matrix)
        # Pointcloud still rotated incorrectly
        #additional_transform = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        #pcd = pcd.transform(additional_transform)
        pcd_data = np.asarray(new_pcd.points) / 1000
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = Header()
        point_cloud_msg.header.frame_id = link_name
        fields =[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(pcd_data)
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 3 * 4
        point_cloud_msg.row_step = point_cloud_msg.point_step * len(pcd_data)
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = bytearray(pcd_data.astype('float32').tobytes())

        publisher.publish(point_cloud_msg)
    #     self.dae_paths_ = glob.glob(self.dae_main_path_ + '/*.dae')
    #     self.publishers_ = []
    #     self.timers_ = []
    #     timer_period = 0.5
    #     for dae_path in self.dae_paths_:
    #         ur5_part = dae_path[dae_path.rfind('/')+1:dae_path.rind('.')]
    #         publisher = self.create_publisher(PointCloud2,ur5_part+'_pointcloud',10)
    #         self.publishers_.append(publisher)
    #         timer = self.create_timer(timer_period,partial(self.timerCallback,param1=dae_path))

    #     self.ur5_parts_ = ['base','forearm','shoulder','upperarm','wrist1','wrist2','wrist3']
    #     self.publishers_ = []
    #     self.timers_ = []
    #     timer_period = 0.5
    #     for ur5_part in self.ur5_parts_:
    #         publisher = self.create_publisher(PointCloud2,ur5_part+'_pointcloud',10)
    #         self.publishers_.append(publisher)
    #         timer = self.create_timer(timer_period,partial(self.timerCallback,param1=ur5_part))
    #     self.publisher_ = self.create_publisher(PointCloud2, '/base_pointcloud', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0
        
    #     print(glob.glob(self.dae_main_path_ + '/*.dae'))
    #     exit()
    #     self.base_filepath_ = "/home/benchturtle/cross_embodiment_ws/src/gazebo_env/meshes/ur5e/visual/base.dae"

    # def timer_callback(self):
    #     mesh = trimesh.load(self.base_filepath_)
    #     vertices = None
    #     for item in mesh.geometry:
    #         if vertices is None:
    #             vertices = mesh.geometry[item].vertices
    #         else:
    #             vertices = np.concatenate((vertices,mesh.geometry[item].vertices),axis=0)

    #     # Create a visualization window
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(vertices)

    #     pcd_data = np.asarray(pcd.points) / 1000
    #     np.savetxt('data.csv',pcd_data,delimiter=", ")
    #     point_cloud_msg = PointCloud2()
    #     point_cloud_msg.header = Header()
    #     point_cloud_msg.header.frame_id = 'base_link_inertia'
    #     fields =[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #     PointField(name='z', offset=4, datatype=PointField.FLOAT32, count=1),
    #     PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
    #     ]
    #     point_cloud_msg.height = 1
    #     point_cloud_msg.width = len(pcd_data)
    #     point_cloud_msg.fields = fields
    #     point_cloud_msg.is_bigendian = False
    #     point_cloud_msg.point_step = 3 * 4
    #     point_cloud_msg.row_step = point_cloud_msg.point_step * len(pcd_data)
    #     point_cloud_msg.is_dense = True
    #     point_cloud_msg.data = bytearray(pcd_data.astype('float32').tobytes())

    #     self.publisher_.publish(point_cloud_msg)
    #     self.i += 1

def main(args=None):
    rclpy.init(args=args)

    point_cloud_publisher = PointCloudPublisher()

    rclpy.spin(point_cloud_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()