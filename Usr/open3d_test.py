# -*- coding = utf-8 -*-
# @Time : 2021/11/29 13:21
# @Author : 戎昱
# @File : open3d_test.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import open3d as o3d
import numpy as np
import os

def parse_bin(lidar_file):
    return np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

def semantics_to_colors(semantics):
    colors = np.zeros((semantics.shape[0], 3))
    # for id in trainId2label:
    #     label = trainId2label[id]
    #     if id == 255 or id == -1:
    #         continue
    #
    #     color = label.color
    #     indices = semantics == id
    #     colors[indices] = (color[0] / 255, color[1] / 255, color[2] / 255)

    return colors

root_dir = r'G:\PP\carla\training\velodyne'
demo_file = os.path.join(root_dir,'a_000060.bin')

pcd = o3d.geometry.PointCloud()
lidar = parse_bin(demo_file)
pointcloud = lidar[:,:3]
semantics  = lidar[:,3]
colors = semantics_to_colors(semantics)

pcd.points = o3d.utility.Vector3dVector(pointcloud)
# pcd.colors = o3d.utility.Vector3dVector(colors)
# points = parse_bin(demo_file)
# o3d.io.write_point_cloud(r'G:\Carla_Recorder\Lidar_Recorder\test'+ r"\1_002250_copy.ply",points)
o3d.visualization.draw_geometries([pcd],width=800,height=600)