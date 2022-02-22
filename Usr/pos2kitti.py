
# -*- coding = utf-8 -*-
# @Time : 2022/1/29 21:27
# @Author : 戎昱
# @File : pos2kitti.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import math, os
from tqdm import tqdm

pos_path = r'G:\Carla_Recorder\tmp'
save_path = r'G:\PP\carla\training\label_2'

def _length(A,B):
    return math.sqrt(pow(A[0]-B[0],2)+pow(A[1]-B[1],2))

def midPoint(A,C):
    return (0.5*(A[0]+C[0]),0.5*(A[1]+C[1]))



def pos2kitti(path):
    pos = pos_path + '/' + path
    save = save_path + '/' + path
    with open(pos,'r') as f:
        lines = f.readlines()
        labels = []
        for line in lines:
            line.strip('\n')
            data = line.split(' ')
            if data[0] == 'Car':
                cat_id = 0
            else:
                cat_id = 1
            bbox = data[21:]
            A = (float(bbox[0]), float(bbox[2]))
            B = (float(bbox[3]), float(bbox[5]))
            C = (float(bbox[6]), float(bbox[8]))
            # print(bbox)
            # print(A[1]-B[1]) # 判断方向
            l, w, h = _length(A,B),_length(B,C), float(bbox[1])-float(bbox[13])
            mid = midPoint(A,C)
            if cat_id == 0:
                x, y, z = mid[0], h - 1, mid[1]
            else:
                x, y, z = mid[0], float(bbox[1]), mid[1]
            # print(A,B,C)
            # print(l, w, h)
            # print(x,y,z)
            #
            # print(A[1]-B[1],A[0]-B[0])

            theta = math.atan2(A[1]-B[1],A[0]-B[0])
            # ry = theta*360/math.pi if theta>0 else (360+theta*360/math.pi)
            ry = theta
            # print(ry)
            # 还差个yaw角

            # print(' ')
            object_label = [cat_id, z, x, y, h, w, l, ry]
            # print(object_label)
            labels.append(object_label)

    with open(save,'w') as f:
        for label in labels:
            f.write(str(label[0]))
            f.write(' ')
            f.write(str(format(label[1], '.4f')))
            f.write(' ')
            f.write(str(format(label[2], '.4f')))
            f.write(' ')
            f.write(str(format(label[3], '.4f')))
            f.write(' ')
            f.write(str(format(label[4], '.4f')))
            f.write(' ')
            f.write(str(format(label[5], '.4f')))
            f.write(' ')
            f.write(str(format(label[6], '.4f')))
            f.write(' ')
            f.write(str(format(label[7], '.4f')))
            f.write('\n')


# filenames = sorted(
#     [filename for filename in os.listdir(pos_path)]
# )
#
# for filename in tqdm(filenames):
#     pos2kitti(filename)
# pos2kitti(pos_path)