# -*- coding = utf-8 -*-
# @Time : 2022/2/4 11:22
# @Author : 戎昱
# @File : kittiDelete.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os
import shutil
root_dir = r"G:\PP\carla\training"

label_path = os.path.join(root_dir,'label_2')
cam_path = os.path.join(root_dir,'image_2')
sem_path = os.path.join(root_dir,'semantic')
lidar_path = os.path.join(root_dir,'velodyne')
pp_path = os.path.join(root_dir,'paintedVelodyne')

shutil.rmtree(label_path)
shutil.rmtree(cam_path)
shutil.rmtree(sem_path)
shutil.rmtree(lidar_path)
shutil.rmtree(pp_path)

os.mkdir(label_path)
os.mkdir(cam_path)
os.mkdir(sem_path)
os.mkdir(lidar_path)
os.mkdir(pp_path)

print("Delete success!")