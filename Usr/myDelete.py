# -*- coding = utf-8 -*-
# @Time : 2021/11/23 15:31
# @Author : 戎昱
# @File : myDelete.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os
import shutil

txt_file = "G:\Carla_Recorder\Position_Recorder"
cam_path = "G:\Carla_Recorder\Cam_Recorder"
sem_path = "G:\Carla_Recorder\semantic_Recorder"
lid_path = "G:\Carla_Recorder\Lidar_Recorder"
laneline_file = r"G:\Carla_Recorder\Laneline_Recorder"
dir = r"\test"

shutil.rmtree(txt_file + dir)
shutil.rmtree(cam_path + dir)
shutil.rmtree(sem_path + dir)
shutil.rmtree(lid_path + dir)
shutil.rmtree(laneline_file + dir)

os.mkdir(txt_file + dir)
os.mkdir(cam_path + dir)
os.mkdir(sem_path + dir)
os.mkdir(lid_path + dir)
os.mkdir(laneline_file + dir)

print("Delete success!")
