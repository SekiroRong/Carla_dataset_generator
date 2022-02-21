# -*- coding = utf-8 -*-
# @Time : 2022/2/3 10:17
# @Author : 戎昱
# @File : makeImageSets.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os
import random
import numpy as np
video_path = r'G:\PP\carla\training'

images_dir = os.path.join(video_path, 'image_2')
txt_dir = r'G:\PP\carla\ImageSets\train.txt'
txt_dir2 = r'G:\PP\carla\ImageSets\test.txt'
txt_dir3 = r'G:\PP\carla\ImageSets\trainval.txt'
txt_dir4 = r'G:\PP\carla\ImageSets\val.txt'

def makeImageSets(renew = True):
    mode = 'w' if renew else 'a'
    images_filenames = sorted(
        [os.path.join(images_dir, filename) for filename in os.listdir(images_dir)]
    )

    length = len(images_filenames)
    train_length = int(0.8 * length)
    val_length = length - train_length

    print(length)

    train_filenames = np.random.choice(images_filenames, train_length, replace=False)

    val_filenames = list(set(images_filenames)^set(train_filenames))

    print(len(train_filenames))
    print(len(val_filenames))

    with open(txt_dir,mode) as f:
        for img in train_filenames:
        # for img in images_filenames:
            img_name = img[29:-4]
            # print(img_name)
            f.write(img_name)
            f.write('\n')

    # with open(txt_dir2,mode) as f:
    #     for img in images_filenames:
    #         img_name = img[29:].strip('.jpg')
    #         # print(img_name)
    #         f.write(img_name)
    #         f.write('\n')

    with open(txt_dir3,mode) as f:
        for img in images_filenames:
            img_name = img[29:-4]
            # print(img_name)
            f.write(img_name)
            f.write('\n')

    with open(txt_dir4,mode) as f:
        for img in val_filenames:
            img_name = img[29:-4]
            # print(img_name)
            f.write(img_name)
            f.write('\n')

# makeImageSets()