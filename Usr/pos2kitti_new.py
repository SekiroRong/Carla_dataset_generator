# -*- coding = utf-8 -*-
# @Time : 2022/3/9 17:41
# @Author : 戎昱
# @File : pos2kitti_new.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import math, os
from tqdm import tqdm
from config import label_path, tmp_path

def _length(A,B):
    return math.sqrt(pow(A[0]-B[0],2)+pow(A[1]-B[1],2))

def midPoint(A,C):
    return (0.5*(A[0]+C[0]),0.5*(A[1]+C[1]))


def pos2kitti(path, real_kitti=False):
    pos = tmp_path + '/' + path
    save = label_path + '/' + path
    with open(pos,'r') as f:
        lines = f.readlines()
        labels = []
        for line in lines:
            line.strip('\n')
            data = line.split(' ')

            bbox2d = data[17:21]
            bbox = data[21:]
            A = (float(bbox[0]), float(bbox[2]))
            B = (float(bbox[3]), float(bbox[5]))
            C = (float(bbox[6]), float(bbox[8]))
            # print(bbox)
            # print(A[1]-B[1]) # 判断方向
            l, w, h = _length(A,B),_length(B,C), float(bbox[1])-float(bbox[13])
            if data[0] != 'Car':
                l += 0.2
                w += 0.2
            mid = midPoint(A,C)
            x, y, z = mid[0], -float(bbox[1]), mid[1]

            theta = math.atan2(A[1]-B[1],A[0]-B[0])

            ry = theta

            object_label = [data[0], bbox2d[0], bbox2d[1], bbox2d[2], bbox2d[3], h, w, l, z, x, y, ry]
            labels.append(object_label)

    with open(save,'w') as f:
        for label in labels:
            f.write(label[0])
            f.write(' ')
            f.write('0.00 0 0.00 ')
            f.write(label[1])
            f.write(' ')
            f.write(label[2])
            f.write(' ')
            f.write(label[3])
            f.write(' ')
            f.write(label[4])
            f.write(' ')
            f.write(str(format(label[5], '.4f')))
            f.write(' ')
            f.write(str(format(label[6], '.4f')))
            f.write(' ')
            f.write(str(format(label[7], '.4f')))
            f.write(' ')
            f.write(str(format(label[8], '.4f')))
            f.write(' ')
            f.write(str(format(label[9], '.4f')))
            f.write(' ')
            f.write(str(format(label[10], '.4f')))
            f.write(' ')
            f.write(str(format(label[11], '.4f')))
            f.write('\n')