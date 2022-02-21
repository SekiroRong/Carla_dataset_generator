# -*- coding = utf-8 -*-
# @Time : 2021/11/21 19:52
# @Author : 戎昱
# @File : jpg2mp4.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import cv2
import glob, os
from tqdm import tqdm

from ply2bev import ply2fv
from position2bev import pos2fv
from generateSegLabel import ll2fv
from pos2kitti import pos2kitti
from makeImageSets import makeImageSets

from config import status, kitti

txt_path = r"G:\Carla_Recorder\Position_Recorder" + '/' + status
rgb_path = r"G:\Carla_Recorder\Cam_Recorder" + '/' + status
semantic_path = r"G:\Carla_Recorder\semantic_Recorder" + '/' + status
lidar_path = r"G:\Carla_Recorder\Lidar_Recorder" + '/' + status
save_path = r"G:\Carla_Recorder\videos"

if kitti:
    rgb_path = r'G:\PP\carla\training\image_2'
    lidar_path = r'G:\PP\carla\training\velodyne'

Lidar_Project = True
Semantic_Project = True
BBox_3D_Project = True
LaneLine_Project = False




save_Tmage_path = False

save2dataset = True

save_video = True

save_3dbbox_path = r"G:\Carla_Dataset\3Dbbox" + '/' + status
save_laneline_path = r"G:\Carla_Dataset\LaneLine" + '/' + status
save_drivearea_path = r"G:\Carla_Dataset\DriveableArea" + '/' + status
save_img_path = r"G:\Carla_Dataset\Image" + '/' + status + "\images.txt"

if kitti:
    save_3dbbox_path = r'G:\Carla_Recorder\tmp'

round = 'm'

Renew = True


def resize(img_array, align_mode):
    _height = len(img_array[0])
    _width = len(img_array[0][0])
    for i in range(1, len(img_array)):
        img = img_array[i]
        height = len(img)
        width = len(img[0])
        if align_mode == 'smallest':
            if height < _height:
                _height = height
            if width < _width:
                _width = width
        else:
            if height > _height:
                _height = height
            if width > _width:
                _width = width

    for i in range(0, len(img_array)):
        img1 = cv2.resize(img_array[i], (_width, _height), interpolation=cv2.INTER_CUBIC)
        img_array[i] = img1

    return img_array, (_width, _height)


def images_to_video(path):
    img_array = []

    with open(save_img_path,'a') as f:
        for filename in tqdm(glob.glob(path + '/*.jpg')):
            if kitti:
                tmp = filename[29:]
            else:
                tmp = filename[37:]
            # print(tmp)
            # while 1:
            #     a= 1
            if tmp[0] == round:
                if save_Tmage_path:
                    f.write(filename)
                    f.write('\n')
                if kitti:
                    file = filename[29:]
                else:
                    file = filename[37:]
                print(file)
                # while 1:
                #     a= 1
                try:
                    img = cv2.imread(filename)
                except:
                    print('libpng error')
                    continue

                if img is None:
                    print(filename + " is error!")
                    continue
                # if Lidar_Project:
                #     img = ply2fv(file,img)

                if BBox_3D_Project:
                    img = pos2fv(file, img, save=save2dataset, save_path=save_3dbbox_path)

                if LaneLine_Project:
                    img = ll2fv(file, img, save=save2dataset, save_path1=save_laneline_path, save_path2=save_drivearea_path)

                img_array.append(img)
                # cv2.imshow("img", img)
                # cv2.waitKey(0)

    if kitti:
        print('pos2kitti')
        filenames = sorted(
            [filename for filename in os.listdir(save_3dbbox_path)]
        )

        for filename in tqdm(filenames):
            pos2kitti(filename)

        print('makeImageSets')
        makeImageSets(renew=Renew)
    # 图片的大小需要一致
    # img_array, size = resize(img_array, 'largest')
    if save_video:
        fps = 5
        out = cv2.VideoWriter(save_path + '/' + status + '_' + round + '.mp4', cv2.VideoWriter_fourcc(*'DIVX'), fps, (1280,720))

        for i in tqdm(range(len(img_array))):
            out.write(img_array[i])
        out.release()


def main():
    print(rgb_path)
    images_to_video(rgb_path)


if __name__ == "__main__":
    main()