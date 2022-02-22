# -*- coding = utf-8 -*-
# @Time : 2021/12/11 11:57
# @Author : 戎昱
# @File : position2bev.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import os
import numpy as np
import cv2
import math
from findBoundingRect import findVehicleBoundingRect,findWalkerBoundingRect,findVehicleBoundingRect_plus,findWalkerBoundingRect_plus
from config import status
from depth_cam_utils import from_depth_get_bbox,parse_depth_img
from ply2bev import compute

scale = 1
bev_size = 300*scale
hero_id = '24'

from config import kitti,txt_path,rgb_path,semantic_path,depth_path

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(128,0,0),(0,128,0),(0,0,128),(128,128,0),
          (128,0,128),(0,128,128),(0,128,128),(0,0,205),(0,0,128),(255,215,0),(255,215,0),(255,215,0),(255,215,0),
          (255,215,0),(255,215,0),(255,215,0),(255,215,0),(255,215,0),(255,215,0),(255,215,0),(255,215,0)]

Vehicles_color = (0,0,142)
Pedestrian_color = (220,20,60)
Vehicles = np.array([0, 0, 142])
Pedestrian = np.array([220, 20, 60])

line_width = 2

# 语义分割与rgb叠加
def sem2fv(sem_img,img):
    print('under construction')


def containAllRects(rects):
    min_x,min_y = 1279,1279
    max_x,max_y = 0,0
    for rect in rects:
        if rect[0] < min_x:
            min_x = rect[0]
        if rect[1] < min_y:
            min_y = rect[1]
        if rect[2] > max_x:
            max_x = rect[2]
        if rect[3] > max_y:
            max_y = rect[3]
    return [min_x,min_y,max_x,max_y]


def bbox3dTo2d_plus(sem,points,type):
    b2d = [max(min(points[0][0], points[1][0], points[2][0], points[3][0]),0),
           max(min(points[4][1], points[5][1], points[6][1], points[7][1]),0),
           min(max(points[4][0], points[5][0], points[6][0], points[7][0]),1279),
           min(max(points[0][1], points[1][1], points[2][1], points[4][1]),719)]
    if b2d[1] == b2d[3] or b2d[0] == b2d[2]:
        return [0, 0, 0, 0]
    sem_seg = sem[int(b2d[1]):int(b2d[3]),int(b2d[0]):int(b2d[2])]

    w,h = int(b2d[3]) - int(b2d[1]),int(b2d[2]) - int(b2d[0])

    if type == 'Car':
        sem_rects = findVehicleBoundingRect_plus(sem_seg)
    else:
        sem_rects = findWalkerBoundingRect_plus(sem_seg)
    if len(sem_rects):
        sem_rect = containAllRects(sem_rects)
        x_l = sem_rect[0]
        x_r = h - sem_rect[2]
        if not (b2d[0] == 0 or b2d[0] + h == 1279):
            if x_l == 0:
                x_l = x_r
            elif x_r == 0:
                x_r = x_l
        final_b2d = [b2d[0] + x_l, b2d[1] + sem_rect[1], b2d[0] + h - x_r,b2d[1] + sem_rect[3]]
        return final_b2d
    else:
        return [0,0,0,0]




# pos转鸟瞰图
def pos2bev(pos_data):

    global bev_size
    img = np.zeros((bev_size, bev_size,3), np.uint8)
    mid_x, mid_y = int(img.shape[0] / 2), int(img.shape[1] / 2)
    for vehicle in pos_data:
        id = vehicle[0]
        location = [scale*float(vehicle[1]),scale*float(vehicle[2]),scale*float(vehicle[3])]
        extent = [scale*float(vehicle[4]),scale*float(vehicle[5]),scale*float(vehicle[6])]
        bbox =[(mid_x + round(location[0] + extent[0]), mid_y + round(location[1] + extent[1])), (mid_x + round(location[0] - extent[0]), mid_y + round(location[1] + extent[1])),
               (mid_x + round(location[0] - extent[0]), mid_y + round(location[1] - extent[1])), (mid_x + round(location[0] + extent[0]), mid_y + round(location[1] - extent[1]))]

        if id == hero_id:
            color = colors[2]
            size = 1
            print('got_hero')
        else:
            color = (255,255,255)
            size = 0

        cv2.circle(img,(max(min(mid_x + round(location[0]), bev_size - 1), 0),max(min(mid_y + round(location[1]), bev_size - 1), 0)),size,color)
        cv2.line(img, bbox[0], bbox[1], color)
        cv2.line(img, bbox[1], bbox[2], color)
        cv2.line(img, bbox[2], bbox[3], color)
        cv2.line(img, bbox[3], bbox[0], color)
        # img[max(min(mid_x + round(location[0]), bev_size - 1), 0)][max(min(mid_y + round(location[1]), bev_size - 1), 0)] = 255
    return img

def _compute_space(point):
    b2d = [max(min(point[0][0], point[1][0], point[2][0], point[3][0]),0),
           max(min(point[4][1], point[5][1], point[6][1], point[7][1]),0),
           min(max(point[4][0], point[5][0], point[6][0], point[7][0]),1279),
           min(max(point[0][1], point[1][1], point[2][1], point[4][1]),719)]
    space = (b2d[2]-b2d[0]) * (b2d[3]-b2d[1])
    return space

def _isInside(rect,point):
    # x, y = int((point[0][0] + point[6][0]) / 2), int((point[0][1] + point[6][1]) / 2)
    x1, y1 = int((point[0][0] + point[4][0]) / 2), int((point[0][1] + point[4][1]) / 2)
    x2, y2 = int((point[1][0] + point[5][0]) / 2), int((point[1][1] + point[5][1]) / 2)
    x3, y3 = int((point[2][0] + point[6][0]) / 2), int((point[2][1] + point[6][1]) / 2)
    x4, y4 = int((point[3][0] + point[7][0]) / 2), int((point[3][1] + point[7][1]) / 2)
    if (rect[0][0] < x1 < rect[1][0] and rect[0][1] < y1 < rect[1][1]) or (rect[0][0] < x2 < rect[1][0] and rect[0][1] < y2 < rect[1][1]) or(rect[0][0] < x3 < rect[1][0] and rect[0][1] < y3 < rect[1][1]) or(rect[0][0] < x4 < rect[1][0] and rect[0][1] < y4 < rect[1][1]) :
        # if (_compute_space(point) > 0.1 * (rect[1][0] - rect[0][0])  * (rect[1][1] - rect[0][1])) and (min(max(point[0][1], point[1][1], point[2][1], point[4][1]),719) - max(min(point[4][1], point[5][1], point[6][1], point[7][1]),0)) > 10:
        if (min(max(point[0][1], point[1][1], point[2][1], point[4][1]),719) - max(min(point[4][1], point[5][1], point[6][1], point[7][1]),0)) > 10:
            return True
        else:
            return False
    else:
        return False


def iou_2d_plus(points1, points2):
    '''
        box [x1,y1,x2,y2]   分别是两对角定点的坐标
    '''
    box1 = points1
    box2 = points2

    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    area_sum = area1 + area2
    small_area = min(area1,area2)
    scale = 1
    if area1 > area2:
        scale = 1
    else:
        scale = 1

    # 计算重叠部分 设重叠box坐标为 [x1,y1,x2,y2]
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    if x1 >= x2 or y1 >= y2 :
        # print('not chongdie')
        return 0, 0
    else:
        inter_area = (x2 - x1) * (y2 - y1)
    return inter_area * 1.0 / (area_sum - inter_area), scale * inter_area * 1.0 / (small_area)

def isInsight(sem_img,point):
    x,y = int((point[0][0]+point[6][0])/2),int((point[0][1]+point[6][1])/2)
    if x<0 or x >1279:
        return False
    else:
        if sem_img[y][x].all() == Vehicles.all():
            # 面积
            # space = _compute_space(point)
            # space1 = abs(point[6][0] - point[0][0]) * abs(point[6][1] - point[0][1])
            # space2 = abs(point[7][0] - point[1][0]) * abs(point[7][1] - point[1][1])
            # 高度
            height = max(abs(point[4][1] - point[0][1]),abs(point[6][1] - point[2][1]))
            if _compute_space(point) > 100*100 and height > 100:
                return True
            else:
                return False
        else:
            return False

def midPointDistance(A,C):
    return (0.25*(A[0]+C[0])*(A[0]+C[0])+0.25*(A[1]+C[1])*(A[1]+C[1]))
def midPoint(A,C):
    return (0.5*(A[0]+C[0]),0.5*(A[1]+C[1]))
def pos2fv(path,img, save = False, save_path = ''):
    depth_img = parse_depth_img(path)
    txt = path.replace('.jpg', '.txt')
    txt = txt_path + '/' + txt
    pos_data = []
    with open(txt, "r") as f:
        for line in f.readlines():
            line = line.strip('\n')  # 去掉列表中每一个元素的换行符
            pos_data.append(line.split(' '))

    sem = path.replace('.txt','.jpg')
    sem = semantic_path + '/' + sem
    sem_img = cv2.imread(sem, cv2.IMREAD_COLOR)

    vehicle_rects = findVehicleBoundingRect(sem_img)
    walker_rects = findWalkerBoundingRect(sem_img)
    rects_occupied = [[] for i in range(len(vehicle_rects))]

    if save == True:
        txt = path.replace('.jpg', '.txt')
        save_txt = save_path + '/' + txt

    with open(save_txt,'w') as f:
        # print(len(vehicle_rects))
        for index, actor in enumerate(pos_data):
            bbox = actor[17:]
            A = (float(bbox[0]), float(bbox[2]))
            # B = (float(bbox[3]), float(bbox[5]))
            C = (float(bbox[6]), float(bbox[8]))

            MidPointDistance = midPointDistance(A,C)
            if MidPointDistance > 2500:
                # print(midPoint(A, C))
                # print('skip_distance')
                continue
            mid = midPoint(A, C)
            h = float(bbox[1]) - float(bbox[13])
            if actor[0] == '0':
                x, y, z = mid[0], h/2 - 1, mid[1]
            else:
                x, y, z = mid[0], float(bbox[1]) - 0.5 * h, mid[1]

            if y > 0:
                y *= -1
            if x == 0:
                continue

            mid, dis = compute(z, x, y)
            if z < 0 or z > 55:
                continue
            # print(mid)
            cv2.circle(img, mid, 2, (0,0,255), -1)
            # if MidPointDistance > 2500:
            #     # print(midPoint(A, C))
            #     # print('skip_distance')
            #     continue

            if mid[0] <0 or mid[0] > 1219:
                cv2.putText(img, "skip_boundry", mid, cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 255), 1)
                # print('skip_boundry')
                continue
            # # print(sem_img[mid[1]][mid[0]][0])
            # if sem_img[mid[1]][mid[0]][0] != 142 and actor[0] == '0':
            #     cv2.putText(img, "skip_car", mid, cv2.FONT_HERSHEY_SIMPLEX, 1,
            #                 (0, 0, 255), 1)
            #     # print(sem_img[mid[1]][mid[0]])
            #     # print('skip_car')
            #     continue
            #
            # if sem_img[mid[1]][mid[0]][0] != 60 and actor[0] == '1':
            #     cv2.putText(img, "skip_ped", mid, cv2.FONT_HERSHEY_SIMPLEX, 1,
            #                 (0, 0, 255), 1)
            #     # print(sem_img[mid[1]][mid[0]])
            #     # print('skip_ped')
            #     continue

            d_p = depth_path + '/' + path
            d_img = cv2.imread(d_p)

            d_img = cv2.cvtColor(d_img, cv2.COLOR_BGR2GRAY)
            dis_bias = z - 4*d_img[mid[1]][mid[0]]
            if actor[0] == '0':
                if dis_bias > 4.5 or dis_bias < -2:
                    cv2.putText(img, str(dis_bias), mid, cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 0, 255), 1)
                    print(dis_bias)
                    continue
            if actor[0] == '1':
                if dis_bias > 3 or dis_bias < -3:
                    cv2.putText(img, str(dis_bias), mid, cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 0, 255), 1)
                    print(dis_bias)
                    continue
            # if d_img[mid[1]][mid[0]]/dis < 1 and dis > 8:
            #     # print('Unmatch')
            #     continue
            if actor[0] == '0':
                point = []
                for i in range(8):
                    point.append((int(scale*int(actor[2*i+1])),int(scale*int(actor[2*i+2]))))

                for i in range(len(vehicle_rects)):
                    if _isInside(vehicle_rects[i],point):
                        rects_occupied[i].append(index)

                rects_occupied[i] = list(set(rects_occupied[i]))
                # Deprecated
                        # if len(rects_occupied[i]) == 0:
                        #     rects_occupied[i].append(index)
                        #     # cur = bbox3dTo2d_plus(sem_img, point, type='Car')
                        #     # cur = from_depth_get_bbox(depth_img, cur)
                        #     # print(index,cur)
                        #
                        # else:
                        #     # skip = True
                        #     count = 0
                        #     target_indexs = []
                        #     for target_index,target_vehicle_index in enumerate(rects_occupied[i]):
                        #         target_point = []
                        #         target_vehicle = pos_data[target_vehicle_index]
                        #         for j in range(8):
                        #             target_point.append((int(scale * int(target_vehicle[2 * j + 1])), int(scale * int(target_vehicle[2 * j + 2]))))
                        #
                        #         bbox = target_vehicle[17:]
                        #         A = (float(bbox[0]), float(bbox[2]))
                        #         # B = (float(bbox[3]), float(bbox[5]))
                        #         C = (float(bbox[6]), float(bbox[8]))
                        #
                        #         targetDistance = midPointDistance(A, C)
                        #
                        #         cur = bbox3dTo2d_plus(sem_img, point, type='Car')
                        #         # cur = from_depth_get_bbox(depth_img, cur)
                        #         # if cur[3] - cur[1] < 5:
                        #         #     break
                        #         pre = bbox3dTo2d_plus(sem_img, target_point, type='Car')
                        #         # pre = from_depth_get_bbox(depth_img, pre)
                        #         if cur == [0,0,0,0]:
                        #             break
                        #         IOU, IOS = iou_2d_plus(cur,pre)
                        #         # print(index,cur,target_index,pre)
                        #         # print(IOU,IOS)
                        #         # rects_occupied[i].append(index)
                        #         # break
                        #         if IOU < 1 and IOS < 0.7:
                        #             count += 1
                        #             # cv2.putText(img,str(IOS), point[7],cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1,(255, 255, 255), line_width)
                        #             # if IOS < -0.7:
                        #             #     rects_occupied[i][target_index] = index
                        #             if count == len(rects_occupied[i]):
                        #                 rects_occupied[i].append(index)
                        #                 rects_occupied[i] = list(set(rects_occupied[i]))
                        #                 break
                        #             # break
                        #         else:
                        #             if MidPointDistance < targetDistance:
                        #                 rects_occupied[i][target_index] = index
                        #             break
                                    # sp1,sp2 = _compute_space(point),_compute_space(target_point)
                                    # if sp1 > sp2:
                                    #     count += 1
                                    #     target_indexs.append(target_index)
                                    #     # rects_occupied[i][target_index] = index
                                    # if count == len(rects_occupied[i]):
                                    #     for target_index in target_indexs:
                                    #         rects_occupied[i][target_index] = index
                                    #     target_indexs = []


                                    # if not (2*sp1 < sp2):
                                    #     count += 1
                                    # else:
                                    #     count += 1
                                        # skip = False
                                        # break

                            # rects_occupied[i]=list(set(rects_occupied[i]))
                        # print(rects_occupied[i])
                        # print(rects_occupied[i])

                        # elif _compute_space(point) > rects_occupied[i][1]:
                        #     rects_occupied[i][0] = index
                        #     rects_occupied[i][1] = _compute_space(point)
                        # break
            elif actor[0] == '1':
                point = []
                for i in range(8):
                    point.append((int(scale * int(actor[2 * i + 1])), int(scale * int(actor[2 * i + 2]))))

                for i in range(len(walker_rects)):
                    if _isInside(walker_rects[i],point):

                        cv2.line(img, point[0], point[1], Pedestrian_color, line_width)
                        cv2.line(img, point[1], point[2], Pedestrian_color, line_width)
                        cv2.line(img, point[2], point[3], Pedestrian_color, line_width)
                        cv2.line(img, point[3], point[0], Pedestrian_color, line_width)
                        cv2.line(img, point[4], point[5], Pedestrian_color, line_width)
                        cv2.line(img, point[5], point[6], Pedestrian_color, line_width)
                        cv2.line(img, point[6], point[7], Pedestrian_color, line_width)
                        cv2.line(img, point[7], point[4], Pedestrian_color, line_width)
                        cv2.line(img, point[0], point[4], Pedestrian_color, line_width)
                        cv2.line(img, point[1], point[5], Pedestrian_color, line_width)
                        cv2.line(img, point[2], point[6], Pedestrian_color, line_width)
                        cv2.line(img, point[3], point[7], Pedestrian_color, line_width)


                        tmp = bbox3dTo2d_plus(sem_img, point, type='Pedestrian')
                        cv2.line(img, (tmp[0], tmp[1]), (tmp[0], tmp[3]), (255, 255, 255), 1)
                        cv2.line(img, (tmp[0], tmp[1]), (tmp[2], tmp[1]), (255, 255, 255), 1)
                        cv2.line(img, (tmp[2], tmp[3]), (tmp[0], tmp[3]), (255, 255, 255), 1)
                        cv2.line(img, (tmp[2], tmp[3]), (tmp[2], tmp[1]), (255, 255, 255), 1)

                        f.write('Pedestrian ')
                        for i in range(8):
                            f.write(str(point[i][0]))
                            f.write(' ')
                            f.write(str(point[i][1]))
                            f.write(' ')

                        for i in range(4):
                            f.write(str(tmp[i]))
                            f.write(' ')

                        for i in range(24):
                            f.write(actor[17+i])
                            f.write(' ')


                        f.write('\n')
                        break

                    # Deprecated

                        # if len(rects_occupied[i]) == 0:
                        #     rects_occupied[i].append(index)
                        # else:
                        #     for target_index,target_vehicle_index in enumerate(rects_occupied[i]):
                        #         target_point = []
                        #         target_vehicle = pos_data[target_vehicle_index]
                        #         for j in range(8):
                        #             target_point.append((int(scale * int(target_vehicle[2 * j + 0])), int(scale * int(target_vehicle[2 * j + 1]))))
                        #
                        #         IOU, IOS = iou_2d_plus(point,target_point)
                        #         # print(IOU,IOS)
                        #
                        #         if IOU < 0.5 and IOS < 0.5:
                        #             # cv2.putText(img,str(IOS), point[7],cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1,(255, 255, 255), line_width)
                        #             rects_occupied[i].append(index)
                        #             break
                        #         else:
                        #             if _compute_space(point) > _compute_space(target_point):
                        #                 rects_occupied[i][target_index] = index

    # print(rects_occupied)
    # print(' ')
    # depth_img = parse_depth_img(path)
    with open(save_txt, 'a') as f:
        for i in range(len(vehicle_rects)):
            if len(rects_occupied[i]) == 0:
                continue
            else:
                for i,vehicle_index in enumerate(rects_occupied[i]):
                    vehicle = pos_data[vehicle_index]
                    point = []
                    for j in range(8):
                        point.append((int(scale * int(vehicle[2 * j + 1])), int(scale * int(vehicle[2 * j + 2]))))

                    # print(point)

                    # line_width = i+1
                    cv2.line(img, point[0], point[1], Vehicles_color, line_width)
                    cv2.line(img, point[1], point[2], Vehicles_color, line_width)
                    cv2.line(img, point[2], point[3], Vehicles_color, line_width)
                    cv2.line(img, point[3], point[0], Vehicles_color, line_width)
                    cv2.line(img, point[4], point[5], Vehicles_color, line_width)
                    cv2.line(img, point[5], point[6], Vehicles_color, line_width)
                    cv2.line(img, point[6], point[7], Vehicles_color, line_width)
                    cv2.line(img, point[7], point[4], Vehicles_color, line_width)
                    cv2.line(img, point[0], point[4], Vehicles_color, line_width)
                    cv2.line(img, point[1], point[5], Vehicles_color, line_width)
                    cv2.line(img, point[2], point[6], Vehicles_color, line_width)
                    cv2.line(img, point[3], point[7], Vehicles_color, line_width)

                    tmp = bbox3dTo2d_plus(sem_img,point,type = 'Car')
                    tmp = from_depth_get_bbox(depth_img,tmp)
                    cv2.line(img, (tmp[0], tmp[1]), (tmp[0], tmp[3]), (255, 255, 255), 1)
                    cv2.line(img, (tmp[0], tmp[1]), (tmp[2], tmp[1]), (255, 255, 255), 1)
                    cv2.line(img, (tmp[2], tmp[3]), (tmp[0], tmp[3]), (255, 255, 255), 1)
                    cv2.line(img, (tmp[2], tmp[3]), (tmp[2], tmp[1]), (255, 255, 255), 1)

                    f.write('Car ')
                    for i in range(8):
                        f.write(str(point[i][0]))
                        f.write(' ')
                        f.write(str(point[i][1]))
                        f.write(' ')

                    for i in range(4):
                        f.write(str(tmp[i]))
                        f.write(' ')

                    for i in range(24):
                        f.write(vehicle[17 + i])
                        f.write(' ')

                    f.write('\n')
    return img
