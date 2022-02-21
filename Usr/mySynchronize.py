# -*- coding = utf-8 -*-
# @Time : 2021/12/12 23:16
# @Author : 戎昱
# @File : mySynchronize.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong

import os,glob
from config import status


txt_path = r"G:\Carla_Recorder\Position_Recorder" + '/' + status
rgb_path = r"G:\Carla_Recorder\Cam_Recorder" + '/' + status
semantic_path = r"G:\Carla_Recorder\semantic_Recorder" + '/' + status
lidar_path = r"G:\Carla_Recorder\Lidar_Recorder" + '/' + status
laneline_file = r"G:\Carla_Recorder\Laneline_Recorder" + '/' + status

rgb = []
for filename in (glob.glob(rgb_path + '/*.jpg')):
    rgb.append(filename.strip(rgb_path).strip('.jpg'))

# print(rgb_path + '/' + rgb[-1] + '.jpg')
os.remove(rgb_path + '/' + rgb[-1] + '.jpg')

sem = []
for filename in (glob.glob(semantic_path + '/*.jpg')):
    sem.append(filename.strip(semantic_path).strip('.jpg'))

txt = []
for filename in (glob.glob(txt_path + '/*.txt')):
    txt.append(filename.strip(txt_path).strip('.txt'))

lidar = []
for filename in (glob.glob(lidar_path + '/*.ply')):
    lidar.append(filename.strip(lidar_path).strip('.ply'))

ll = []
for filename in (glob.glob(laneline_file + '/*.txt')):
    ll.append(filename.strip(laneline_file).strip('.txt'))

print('rgb',len(rgb))
print('sem',len(sem))
print('txt',len(txt))
print('lidar',len(lidar))
print('ll',len(ll))

intersection = list(set(rgb).intersection(sem,lidar))

print('intersection',len(intersection))

ret_list = list(set(rgb)^set(intersection))

print('ret_list',len(ret_list))

for id in ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')
    try:
        os.remove(semantic_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(semantic_path + '/' + id + '.jpg' + ' not found.')
    try:
        os.remove(lidar_path + '/' + id + '.ply')
    except FileNotFoundError:
        print(lidar_path + '/' + id + '.ply' + ' not found.')

ret_list = list(set(sem) ^ set(intersection))
for id in ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')

ret_list = list(set(lidar) ^ set(intersection))
for id in ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')

ret_list = list(set(txt) ^ set(intersection))
for id in ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')

ret_list = list(set(ll) ^ set(intersection))
for id in ret_list:
    try:
        os.remove(rgb_path + '/' + id + '.jpg')
    except FileNotFoundError:
        print(rgb_path + '/' + id + '.jpg' + ' not found.')

print('Synchronize success!')
# os.remove(path)


