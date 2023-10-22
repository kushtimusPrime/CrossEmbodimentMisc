import open3d as o3d
import trimesh
import numpy as np
import matplotlib.pyplot as plt
# Load the .dae file
filename = "/home/benchturtle/cross_embodiment_ws/install/robotiq_description/share/robotiq_description/meshes/visual/ur_to_robotiq_adapter.dae"
mesh_scene = trimesh.load(filename)
mesh = trimesh.util.concatenate(tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                                    for g in mesh_scene.geometry.values()))
# Convert Trimesh to Open3D TriangleMesh
vertices = o3d.utility.Vector3dVector(mesh.vertices)
triangles = o3d.utility.Vector3iVector(mesh.faces)
open3d_mesh = o3d.geometry.TriangleMesh(vertices, triangles)
print(open3d_mesh)
R = np.array([[-1,0,0],[0,0,1],[0,1,0]])
open3d_mesh.rotate(R,[0,0,0])
pcd = open3d_mesh.sample_points_uniformly(number_of_points=20000)
pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) / 1000)
pcd_data = np.asarray(pcd.points)
# Create an Open3D visualization window
o3d.visualization.draw_geometries([pcd])
exit()
o3d_mesh = None
pcds = []
points = None
#R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
#R2 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
print(mesh)
for item in mesh.geometry:
    o3d_mesh = mesh.geometry[item].as_open3d
    print(o3d_mesh.get_center() / 1000)
    pcd = o3d_mesh.sample_points_uniformly(number_of_points=100000)
    if points is None:
        points = np.asarray(pcd.points)
    else:
        points = np.concatenate((points,pcd.points),axis=0)
    pcds.append(pcd)
new_pcd = o3d.geometry.PointCloud()
new_pcd.points = o3d.utility.Vector3dVector(points)
#new_pcd = new_pcd.rotate(R)
#new_pcd = new_pcd.rotate(R2)
new_pcd.points = o3d.utility.Vector3dVector(np.asarray(new_pcd.points) / 1000)
#center_translation = -new_pcd.get_center()
#new_pcd = new_pcd.translate(center_translation)
#print(new_pcd.get_center())


o3d.visualization.draw_geometries([new_pcd,mesh_coordinate_frame])

# mesh = trimesh.load(collada_filepath)
# #print(transform_matrix)
# #mesh = mesh.apply_transform(transform_matrix)
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
# mesh_coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
# o3d.visualization.draw_geometries([pcd,mesh_coordinate_frame])

