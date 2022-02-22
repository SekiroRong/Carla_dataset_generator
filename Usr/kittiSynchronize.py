# -*- coding = utf-8 -*-
# @Time : 2022/2/7 13:47
# @Author : 戎昱
# @File : newSynchronize.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os,glob
from config import status, kitti, rgb_path, semantic2_path, lidar_path, label_path, paintedVelodyne_path

if not kitti:
    print('This script is only working in kitti mode')
    raise RuntimeError

rgb = []
for filename in (glob.glob(rgb_path + '/*.jpg')):
    rgb.append(filename.replace(rgb_path,'')[:-4])

sem = []
for filename in (glob.glob(semantic2_path + '/*.jpg')):
    sem.append(filename.replace(semantic2_path,'')[:-4])

lidar = []
for filename in (glob.glob(lidar_path + '/*.bin')):
    lidar.append(filename.replace(lidar_path,'')[:-4])

p_lidar = []
for filename in (glob.glob(paintedVelodyne_path + '/*.bin')):
    p_lidar.append(filename.replace(paintedVelodyne_path,'')[:-4])

label = []
for filename in (glob.glob(label_path + '/*.txt')):
    label.append(filename.replace(label_path,'')[:-4])

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


for id in rgb_ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')

for id in sem_ret_list:
    try:
        os.remove(semantic2_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(semantic2_path + '/' + id + '.jpg' + ' not found.')

for id in lidar_ret_list:
    try:
        os.remove(lidar_path + '/' + id + '.bin')
    except FileNotFoundError:
        print(lidar_path + '/' + id + '.bin' + ' not found.')

for id in p_lidar_ret_list:
    try:
        os.remove(paintedVelodyne_path + '/' + id + '.bin')
    except FileNotFoundError:
        print(paintedVelodyne_path + '/' + id + '.bin' + ' not found.')

for id in label_ret_list:
    try:
        os.remove(label_path + '/' + id + '.txt')
    except FileNotFoundError:
        print(label_path + '/' + id + '.txt' + ' not found.')

print('Synchronize success!')
