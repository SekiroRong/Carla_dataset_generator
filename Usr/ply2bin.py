# -*- coding = utf-8 -*-
# @Time : 2022/2/3 21:52
# @Author : 戎昱
# @File : ply2bin.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import numpy as np
import os
from tqdm import tqdm
np.set_printoptions(suppress=True)

# Transfrom the ply file into Kitti form(bin file)
def parse_ply(path):
    ply_data = []
    with open(path, "r") as f:
        l_num = 0
        for line in f.readlines():
            l_num += 1
            if l_num > 8:
                line = line.strip('\n')  # 去掉列表中每一个元素的换行符
                ply_data.append(line.split(' '))
    return np.array(ply_data).astype(np.float32)

def ply2bin(path):
    lidar = parse_ply(path)
    # lidar = lidar[:, [1, 2, 0, 3]]
    # lidar[:,0] *= -1
    velodyne_file_new = path[0:-3] + 'bin'
    lidar.tofile(velodyne_file_new)
    os.remove(path)


# root_dir = r'G:\PP\carla\testing\velodyne'
# filenames = sorted(
#     [os.path.join(root_dir,filename) for filename in os.listdir(root_dir)]
# )
# for filename in tqdm(filenames):
#     ply2bin(filename)
# test = r'G:\PP\carla\testing\velodyne\x_000020.ply'
# ply2bin(test)