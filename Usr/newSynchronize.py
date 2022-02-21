# -*- coding = utf-8 -*-
# @Time : 2022/2/7 13:47
# @Author : 戎昱
# @File : newSynchronize.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os,glob
from config import status


image_2 = r"G:\PP\carla\training\image_2"
semantic = r"G:\PP\carla\training\semantic"
velodyne = r"G:\PP\carla\training\velodyne"
label_2 = r"G:\PP\carla\training\label_2"
paintedVelodyne = r"G:\PP\carla\training\paintedVelodyne"

rgb = []
for filename in (glob.glob(image_2 + '/*.jpg')):
    rgb.append(filename[29:-4])

sem = []
for filename in (glob.glob(semantic + '/*.jpg')):
    sem.append(filename[30:-4])
# print(sem)

lidar = []
for filename in (glob.glob(velodyne + '/*.bin')):
    lidar.append(filename[30:-4])

p_lidar = []
for filename in (glob.glob(paintedVelodyne + '/*.bin')):
    p_lidar.append(filename[37:-4])

label = []
for filename in (glob.glob(label_2 + '/*.txt')):
    label.append(filename[29:-4])

# print(label)

#
print('rgb',len(rgb))
print('sem',len(sem))
print('lidar',len(lidar))
print('p_lidar',len(p_lidar))
print('label',len(label))

#
intersection = list(set(rgb).intersection(sem,lidar))
print('intersection',len(intersection))

rgb_ret_list = list(set(rgb)^set(intersection))
print('rgb_ret_list',len(rgb_ret_list))

sem_ret_list = list(set(sem)^set(intersection))
print('sem_ret_list',len(sem_ret_list))

lidar_ret_list = list(set(lidar)^set(intersection))
print('lidar_ret_list',len(lidar_ret_list))

p_lidar_ret_list = list(set(p_lidar)^set(intersection))
print('p_lidar_ret_list',len(p_lidar_ret_list))

label_ret_list = list(set(label)^set(intersection))
print('label_ret_list',len(label_ret_list))

#
for id in rgb_ret_list:
    try:
        os.remove(image_2 + '/' + id + '.jpg')
    except FileNotFoundError:
        print(image_2 + '/' + id + '.jpg' + ' not found.')

for id in sem_ret_list:
    try:
        os.remove(semantic + '/' + id + '.jpg')
    except FileNotFoundError:
        print(semantic + '/' + id + '.jpg' + ' not found.')

for id in lidar_ret_list:
    try:
        os.remove(velodyne + '/' + id + '.bin')
    except FileNotFoundError:
        print(velodyne + '/' + id + '.bin' + ' not found.')

for id in p_lidar_ret_list:
    try:
        os.remove(paintedVelodyne + '/' + id + '.bin')
    except FileNotFoundError:
        print(paintedVelodyne + '/' + id + '.bin' + ' not found.')

for id in label_ret_list:
    try:
        os.remove(label_2 + '/' + id + '.txt')
    except FileNotFoundError:
        print(label_2 + '/' + id + '.txt' + ' not found.')

print('Synchronize success!')
# os.remove(path)