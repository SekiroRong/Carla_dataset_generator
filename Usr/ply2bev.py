# -*- coding = utf-8 -*-
# @Time : 2021/11/27 22:50
# @Author : 戎昱
# @File : ply2bev.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os
import numpy as np
import cv2
import math
import random
from config import status
np.set_printoptions(suppress=True)

sparse = 1
bev_size = 1000
focal_length = 10 # 焦距:默认为10m
z_scale = 360/10 #Lidar Y轴缩放比
y_scale = 640/10 #Lidar X轴缩放比
z_height = 1 # 雷达高度：设定的1m好像
y_offset = 640
z_offset = 360

ply_path = r'G:\Carla_Recorder\Lidar_Recorder' + '/' + status
rgb_path = r'G:\Carla_Recorder\Cam_Recorder' + '/' + status

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255),(128,0,0),(0,128,0),(0,0,128),(128,128,0),
          (128,0,128),(0,128,128),(200,0,0),(0,200,0),(0,0,200),(200,200,0),(200,0,200),(0,200,200),(100,0,0),
          (0,100,0),(0,0,100),(100,100,0),(100,0,100),(0,100,100),(255,215,0),(255,215,0),(255,215,0)]
ply_data = []

def compute(x,y,z):
    dis = math.sqrt(x*x+y*y+z*z)
    try:
        s1 = (y*y + z*z) / (x*x)
    except ZeroDivisionError:
        return (-1,-1), -1
    s2 = y/z
    a = s1*focal_length*focal_length/(s2*s2+1)
    b = math.sqrt(a)
    c = -s2*b
    return (y_offset + round(y_scale*c),z_offset + round(z_scale*b)), dis

def compute_verse(rgb_x,rgb_y):
    b = (rgb_y - z_offset)*1.0/z_scale
    x = b/focal_length
    c = (rgb_x - y_offset)*1.0/y_scale
    try:
        y = c/b
    except ZeroDivisionError:
        y = 100
    # print(x*100,y*100)
    return (int(x*205*2), int(y*10*4))

def distance_O(point1,point2):
    return point1-point2


# lidar to bev map
def ply2bev(ply_data):
    global bev_size
    img = np.zeros((bev_size, bev_size), np.uint8)
    mid_x,mid_y = int(img.shape[0]/2),int(img.shape[1]/2)
    for point in range(ply_data.shape[0]):
        x,y,z = -round(10*float(ply_data[point][0])),round(10*float(ply_data[point][1])),int(float(ply_data[point][2]))
        # print(ply_data[point])
        # z=1
        # if (x < -y  and -y < 0 ) or (-y < -x  and -y > 0): # FOV = 90
        if 1:
            img[max(min(mid_x+x,bev_size-1),0)][max(min(mid_y+y,bev_size-1),0)] = 255
    # return cv2.flip(img,-1)
    return img

# lidar to front view
def ply2fv(path,img):
    # rgb = rgb_path + '/' + path
    ply = path.replace('.jpg','.ply')
    ply = r'G:\PP\carla\training\velodyne' + '/' + ply
    ply_data = []
    with open(ply, "r") as f:
        l_num = 0
        for line in f.readlines():
            l_num += 1
            if l_num > 8:
                line = line.strip('\n')  # 去掉列表中每一个元素的换行符
                ply_data.append(line.split(' '))

    ply_data = np.array(ply_data)
    # print(ply_data.shape)
    ply_data = ply_data.astype(np.float32)
    # print(ply_data)
    # img = cv2.imread(rgb,cv2.IMREAD_COLOR)
    # print(img.shape)
    # mid_x, mid_y = int(img.shape[0] / 2), int(img.shape[1] / 2)
    for point in range(ply_data.shape[0]):
        x, y, z = float(ply_data[point][0]), float(ply_data[point][1]), float(ply_data[point][2])
        if (-x < -y  and -y < 0 ) or (-y < x  and -y > 0): # FOV = 90
            print(-y,z,x,1)
            while 1:
                a = 1
            u,v = new_projection(-y,z,x)
            if u>1279 or u<0 or v > 719 or v < 0:
                continue

            new_pixel = (u,v)

            # print(u,v)
            # new_pixel,dis = compute(x, y, z)
            rd = random.random()
            if rd < sparse:
                cv2.circle(img, new_pixel, 1, colors[int(x/2)], -1)
            # i +=1
    # print(str(i))
    return img

M = [[-640, 0   , 640, 0],
     [0   , -640, 360, 0],
     [0   , 0   , 1  , 0]]
M = np.array(M,dtype=np.float32)

def new_projection(Xc,Yc,Zc):
    XYZ = [[Xc],
           [Yc],
           [Zc],
           [1]]
    UV = np.matmul(M,XYZ)
    # print(UV.shape)

    return int(UV[0][0]/Zc),int(UV[1][0]/Zc)


# root_dir = r'G:\PP\carla\training\velodyne'
# # root_dir = r'G:\kitti\training\velodyne'
# bin = os.path.join(root_dir, 'l_000010.bin')
# lidar = np.fromfile(bin, dtype=np.float32).reshape(-1, 4)
# lidar = lidar[:, [2, 0, 1, 3]]
#
# print(lidar.shape)
# print(lidar)
# img = ply2bev(lidar)
# cv2.imshow("xx.jpg",img)
# cv2.waitKey(0)
# path = "x_000130.jpg"
# jpg_path = r'G:\PP\carla\training\image_2' + '/' + path
# print(jpg_path)
# img = cv2.imread(jpg_path)
# # img = ply2bev(ply_data)
# ply = path.replace('.jpg', '.ply')
# ply = r'G:\PP\carla\training\velodyne' + '/' + ply
# ply_data = []
# with open(ply, "r") as f:
#     l_num = 0
#     for line in f.readlines():
#         l_num += 1
#         if l_num > 8:
#             line = line.strip('\n')  # 去掉列表中每一个元素的换行符
#             ply_data.append(line.split(' '))
#
# ply_data = np.array(ply_data)
# # img = ply2fv(path, img)
# img = ply2bev(ply_data)
# # print(img)
# print(img.shape)
# cv2.imshow("xx.jpg",img)
# cv2.waitKey(0)
# cv2.imwrite('bev16.jpg',img)