# -*- coding = utf-8 -*-
# @Time : 2021/12/28 18:07
# @Author : 戎昱
# @File : config.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os

# ---------------------Generator Parameters------------------------
status = 'train' # train/test/val

round = 'a' # Avoiding the generated data conflicting

carla_map = 'Town01' # Town01,02,03,04,05,06,07,10 available  Town03有上下坡！

weather_index = 0 # change the weather when simulating

# ------------------------Path----------------------------------------

kitti = True
kitti_root = r'G:\PP\carla'
kitti_training = kitti_root + r'\training'

Carla_Recorder_dir = r'G:\Carla_Recorder'

txt_path = Carla_Recorder_dir + '\Position_Recorder' + '/' + status
rgb_path = Carla_Recorder_dir + '\Cam_Recorder' + '/' + status
semantic_path = Carla_Recorder_dir + '\semantic_Recorder' + '/' + status
lidar_path = Carla_Recorder_dir + 'Lidar_Recorder' + '/' + status
depth_path = Carla_Recorder_dir + '\Depth_Recorder' + '/' + status
semantic2_path = Carla_Recorder_dir + '\semantic2_Recorder' + '/' + status
laneline_file = r"G:\Carla_Recorder\Laneline_Recorder" + '/' + status # Deprecated
save_path = Carla_Recorder_dir + r'\videos'

recorder_dir = Carla_Recorder_dir + r'\recording01.log'

if kitti:
    rgb_path = kitti_training + r'\image_2'
    lidar_path = kitti_training + r'\velodyne'
    semantic2_path = kitti_training + r'\semantic'
    label_path = kitti_training + r'\label_2'
    paintedVelodyne_path = kitti_training + r'\paintedVelodyne'

# ----deprecated------
save_3dbbox_path = r"G:\Carla_Dataset\3Dbbox" + '/' + status
save_laneline_path = r"G:\Carla_Dataset\LaneLine" + '/' + status
save_drivearea_path = r"G:\Carla_Dataset\DriveableArea" + '/' + status
save_img_path = r"G:\Carla_Dataset\Image" + '/' + status + "\images.txt"
# --------------------

if kitti:
    save_3dbbox_path = Carla_Recorder_dir + r'\tmp'


def mkdir(path):
    folder = os.path.exists(path)
    if not folder:
        os.makedirs(path)

mkdir(rgb_path)
mkdir(lidar_path)
mkdir(semantic2_path)
mkdir(label_path)
mkdir(semantic_path)
mkdir(txt_path)
mkdir(depth_path)
mkdir(save_path)

