
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



# -----------deprecated--------------------
def pos2kitti(path, real_kitti=False):
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

            bbox2d = data[17:21]
            # print(bbox2d)
            bbox = data[21:]
            A = (float(bbox[0]), float(bbox[2]))
            B = (float(bbox[3]), float(bbox[5]))
            C = (float(bbox[6]), float(bbox[8]))
            # print(bbox)
            # print(A[1]-B[1]) # 判断方向
            l, w, h = _length(A,B),_length(B,C), float(bbox[1])-float(bbox[13])
            if cat_id == 1:
                l += 0.2
                w += 0.2
            mid = midPoint(A,C)
            x, y, z = mid[0], -float(bbox[1]), mid[1]
            # if cat_id == 0:
            #     if real_kitti:
            #         x, y, z = mid[0], -1, mid[1]
            #     else:
            #         x, y, z = mid[0], h - 1, mid[1]
            # else:
            #     if real_kitti:
            #         x, y, z = mid[0], float(bbox[13]), mid[1]
            #     else:
            #         x, y, z = mid[0], float(bbox[1]), mid[1]
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
            object_label = [cat_id, z, x, y, h, w, l, ry, bbox2d[0], bbox2d[1], bbox2d[2], bbox2d[3]]
            # print(object_label)
            labels.append(object_label)

    with open(save,'w') as f:
        for label in labels:
            f.write(str(label[0]))
            f.write(' ')
            # if real_kitti: # just take place
            #     f.write('0.00 0 0.00 0.00 0.00 0.00 0.00 ')
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
            f.write(' ')
            f.write(label[8])
            f.write(' ')
            f.write(label[9])
            f.write(' ')
            f.write(label[10])
            f.write(' ')
            f.write(label[11])
            f.write('\n')

# turn kitti_like_format 2 real_kitti_format
def real_kitti(filename):
    with open(os.path.join(label_path,filename), 'r') as f:
        lines = f.readlines()
        objects = []
        for line in lines:
            object = [] # type  truncated  occluded  alpha  bbox(4 values/dont care)  dimensions(3 values)
                        # location(3 values)  rotation_y
            line.strip('\n')
            data = line.split(' ')
            if data[0] == '0':
                object.append('Car')
            else:
                object.append('Pedestrian')

            object.append('0.00') # truncated(dont care)
            object.append('0') # occluded(dont care)
            object.append('0.00') # alpha(dont care)
            object.append(data[8]) # bbox(dont care)
            object.append(data[9])
            object.append(data[10])
            object.append(data[11][:-1])
            # object.append('0.00 0.00 0.00 0.00')  # bbox(dont care)
            object.append(data[4]) #h
            object.append(data[5]) #w
            object.append(data[6]) #l
            object.append(data[2]) #x
            object.append(data[3]) #y
            object.append(data[1])  # z
            object.append(data[7])

            objects.append(object)

    with open(os.path.join(label_path_kitti,filename), 'w') as f:
        for object in objects:
            for i in range(15):
                f.write(object[i])
                if i == 14:
                    f.write('\n')
                else:
                    f.write(' ')


label_path = r'G:\PP\carla\training\label_2'
label_path_kitti = r'G:\PP\carla\training\label_2_kitti'
#
# filenames = sorted(
#     [filename for filename in os.listdir(label_path)]
# )
#
# for filename in tqdm(filenames):
#     real_kitti(filename)