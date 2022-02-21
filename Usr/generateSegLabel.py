# -*- coding = utf-8 -*-
# @Time : 2021/12/13 21:58
# @Author : 戎昱
# @File : generateSegLabel.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import math
import os,glob
import cv2
import numpy as np
import numba
from numba import jit
from tqdm import tqdm
# from ply2bev import compute_verse
from parseLLtxt import parseLLtxt
from config import status, kitti

RoadLine = np.array([157, 234, 50])
RoadLine_low_hsv = np.array([76, 200, 233])
RoadLine_high_hsv = np.array([77, 201, 234])
Road_low_hsv = np.array([150, 128, 128])
Road_high_hsv = np.array([151, 129, 129])
_RoadLine = (157, 234, 50)
Road 	 = (128, 64, 128)
road = np.array([128, 64, 128],np.uint8)
white    = (255, 255, 255)
black    = (0, 0, 0)

laneline_file = r"G:\Carla_Recorder\Laneline_Recorder" + '/' + status
# txt_path = r"G:\Carla_Recorder\Position_Recorder\test"
# rgb_path = r"G:\Carla_Recorder\Cam_Recorder" + '/' + status
semantic_path = r"G:\Carla_Recorder\semantic_Recorder" + '/' + status
# lidar_path = r"G:\Carla_Recorder\Lidar_Recorder" + '/' + status

LaneLine_path = r"G:\Carla_Dataset\LaneLine" + '/' + status
DriveableArea_path = r"G:\Carla_Dataset\DriveableArea" + '/' + status

# if kitti:


def ll2fv(path,img, save = False, save_path1 = '', save_path2 = ''):
    if save == True:
        txt = path.replace('.jpg', '.txt')
        save_ll = save_path1 + '/' + path
        save_da = save_path2 + '/' + path

    ll_sem = path.strip('.jpg')
    cur_ll = parseLLtxt(ll_sem)

    sem = semantic_path + '/' + path
    sem_img = cv2.imread(sem)
    hsv = cv2.cvtColor(sem_img, cv2.COLOR_RGB2HSV)
    ll_mask = cv2.inRange(hsv, lowerb=RoadLine_low_hsv, upperb=RoadLine_high_hsv)
    mask = cv2.inRange(hsv, lowerb=RoadLine_low_hsv, upperb=RoadLine_high_hsv)
    road_mask = cv2.inRange(hsv, lowerb=Road_low_hsv, upperb=Road_high_hsv)
    road_mask = cv2.bitwise_or(road_mask, mask)

    rect = np.array([[0, 360], [0, 400], [1279, 400], [1279, 360]])
    cv2.fillConvexPoly(mask, rect, 0)

    prewitt1 = np.array([[1, 0, -1]])
    prewitt2 = np.array([[-1, 0, 1]])

    edge1 = cv2.filter2D(mask, -1, prewitt1)
    edge2 = cv2.filter2D(mask, -1, prewitt2)
    edge = cv2.bitwise_or(edge1, edge2)

    bev = np.zeros((400, 400), np.uint8)
    bev = myProject(edge, bev)

    # bev滤波
    kernel = np.ones((3, 3), np.uint8)
    bev = cv2.dilate(bev, kernel)
    bev = cv2.erode(bev, kernel)
    bev = cv2.medianBlur(bev, 5)
    bev = cv2.medianBlur(bev, 5)

    bev = cv2.filter2D(bev, -1, prewitt2)

    lines = []
    bev_lines = []
    contours, hierarchy = cv2.findContours(bev, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # 找到边界坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        # cv2.rectangle(bev,(x,y),(x+w,y+h),100,2)
        # cv2.putText(mask, str(abs(p1[1]-p2[1])), (x+int(w/2), y+int(h/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, 100, 1)
        # 拟合直线
        if h > 5:
            f_end, b_end = findEndPoint(bev, x, y, w, h)
            cv2.line(bev, f_end, b_end, 100, 2)
            bev_lines.append([f_end, b_end])
            try:
                lines.append([compute(f_end[0], f_end[1], -1), compute(b_end[0], b_end[1], -1)])
            except TypeError:
                continue
    # with open(save_ll, 'w') as f:
    #     for line in lines:
    #         point1 = line[0]
    #         point2 = line[1]
    #         # print(point1,point2)
    #         cv2.line(img, point1, point2, _RoadLine, 2)
    #
    #         f.write(str(point1[0]))
    #         f.write(' ')
    #         f.write(str(point1[1]))
    #         f.write(' ')
    #         f.write(str(point2[0]))
    #         f.write(' ')
    #         f.write(str(point2[1]))
    #         f.write('\n')


    p1, p2, p3, p4 = drawDriveableArea(cur_ll, bev_lines)
    # print(p1, p2, p3, p4)
    cv2.line(bev, p1, p2, 170, 1)
    cv2.line(bev, p2, p3, 170, 1)
    cv2.line(bev, p3, p4, 170, 1)
    cv2.line(bev, p4, p1, 170, 1)
    if p1 == (0,0) and p2 == (399,0) and p3 == (399,399)  and p4 == (0,399):
        p1,p2,p3,p4 = (0,0),(1279,0),(1279,719),(0,719)
    else:
        p1, p2, p3, p4 = compute(p1[0], p1[1], -1), compute(p2[0], p2[1], -1), compute(p3[0], p3[1], -1), compute(p4[0],
                                                                                                              p4[1], -1)

    # p2, p3 = lineExpand_vertical([p2, p3])
    # p1, p4 = lineExpand_vertical([p1, p4])

    rect = np.array([p1, p2, p3, p4])
    cv2.fillConvexPoly(mask, rect, 0)

    # cv2.line(mask, p1, p2, 170, 1)
    cv2.line(mask, p2, p3, 170, 1)
    # cv2.line(mask, p3, p4, 170, 1)
    cv2.line(mask, p4, p1, 170, 1)

    # cv2.line(img, p1, p2, _RoadLine, 1)
    cv2.line(img, p2, p3, _RoadLine, 1)
    # cv2.line(img, p3, p4, _RoadLine, 1)
    cv2.line(img, p4, p1, _RoadLine, 1)

    da_blank = np.zeros([720, 1280], np.uint8)
    # cv2.line(da_blank, p1, p2, 255, 1)
    cv2.line(da_blank, p2, p3, 255, 1)
    # cv2.line(mask, p3, p4, 170, 1)
    cv2.line(da_blank, p4, p1, 255, 1)

    # 可驾驶区域
    rect = np.array([[0, 360], [1279, 360]])
    cv2.fillConvexPoly(mask, rect, 128)
    seed = (min(max(int(0.5*(p1[0]+p3[0])),0),1279),min(int(0.5*(p1[1]+p3[1])),719))
    # print(seed)
    da = fill_color_demo(da_blank, road_mask,seed)
    retval,da = cv2.threshold(da, 170, 255, cv2.THRESH_BINARY)
    purple = np.zeros([720, 1280,3],np.uint8)
    purple.fill(255)
    da_rgb = cv2.cvtColor(da, cv2.COLOR_GRAY2RGB)
    da_rgb = cv2.bitwise_and(da_rgb,purple)
    # cv2.imshow("da_rgb", da_rgb)
    # cv2.waitKey(0)
    # img = cv2.bitwise_or(da_rgb,img)
    img = cv2.addWeighted(da_rgb, 0.3, img, 0.7, 0)

    if save:
        # kernel = np.ones((5, 5), np.uint8)
        # for i in range(10):
        #     da_rgb = cv2.dilate(da_rgb, kernel)
        # for i in range(10):
        #     da_rgb = cv2.erode(da_rgb, kernel)
        cv2.imwrite(save_ll, ll_mask)
        cv2.imwrite(save_da, da_rgb)
        # print(ll_mask.shape)
        # print(da_rgb.shape)
    # for i in range(72):
    #     line_y = 360 + i*5
    #     rect = np.array([[0, line_y], [1279, line_y]])
    #     cv2.fillConvexPoly(mask, rect, 0)
    #
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    #     if x + w / 2 < 640:
    #         point1 = compute_verse(x + w, y)
    #         point2 = compute_verse(x, y + h)
    #     else:
    #         point1 = compute_verse(x + w, y + h)
    #         point2 = compute_verse(x, y)
    #     # print(abs(point1[1] - point2[1]))
    #     if abs(point1[1] - point2[1]) > 20:
    #         rect = np.array([[x+1, y], [x + w - 1, y], [x + w - 1 , y + h], [x + 1, y + h]])
    #         cv2.fillConvexPoly(mask, rect, 0)
    # cv2.imshow("fill_color_demo", mask)
    # cv2.waitKey(0)
    # kernel = np.ones((2, 2), np.uint8)
    # mask = cv2.dilate(mask, kernel)
    # # mask = cv2.medianBlur(mask, 3)
    # mask = cv2.dilate(mask, kernel)
    #
    # cv2.imshow("fill_color_demo", mask)
    # cv2.waitKey(0)
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # # cv2.imshow("img", mask)
    # # cv2.waitKey(0)
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    #     if x + w / 2 < 640:
    #         point1 = compute_verse(x + w, y)
    #         point2 = compute_verse(x, y + h)
    #     else:
    #         point1 = compute_verse(x + w, y + h)
    #         point2 = compute_verse(x, y)
    #     # print(abs(point1[1] - point2[1]))
    #     if abs(point1[0] - point2[0]) < 100:
    #         rect = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
    #         cv2.fillConvexPoly(mask, rect, 0)
    #
    # cv2.imshow("img", mask)
    # cv2.waitKey(0)
    #
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # for c in contours:
    #     # 找到边界坐标
    #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    #     # cv2.putText(mask, str(abs(p1[1]-p2[1])), (x+int(w/2), y+int(h/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, 100, 1)
    #     # 拟合直线
        # if h / ((y - 360) + 1) > 0.20:
            # [vx, vy, point_x, point_y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            # # if vy == 0:
            # #     vy = 0.0001
            # k = vx / vy
            #
            # try:
            #     x1 = int(k * (360 - point_y) + point_x)
            #     x2 = int(k * (719 - point_y) + point_x)
            # except OverflowError:
            #     continue
            # cv2.line(img, (x1, 360), (x2, 719), _RoadLine, 3)
            # cv2.line(mask, (x1, 360), (x2, 719), 255, 2)

    # cv2.imshow("fill_color_demo", mask)
    # cv2.waitKey(0)

    # # 可驾驶区域
    # rect = np.array([[0, 360], [1279, 360]])
    # cv2.fillConvexPoly(mask, rect, 128)
    # da = fill_color_demo(mask, road_mask)
    # retval,da = cv2.threshold(da, 170, 255, cv2.THRESH_BINARY)
    # # cv2.imshow("fill_color_demo", da)
    # # cv2.waitKey(0)
    # purple = np.zeros([720, 1280,3],np.uint8)
    # purple.fill(255)
    # da_rgb = cv2.cvtColor(da, cv2.COLOR_GRAY2RGB)
    # da_rgb = cv2.bitwise_and(da_rgb,purple)
    # # cv2.imshow("da_rgb", da_rgb)
    # # cv2.waitKey(0)
    # # img = cv2.bitwise_or(da_rgb,img)
    # img = cv2.addWeighted(da_rgb, 0.3, img, 0.7, 0)
    # # cv2.imshow("fill_color_demo", img)
    # # cv2.waitKey(0)
    return img

def fill_color_demo(image,road_mask,seed = (640,719)):
    # copyImg = image.copy()
    h, w = image.shape[:2]
    mask = np.zeros([h+2, w+2],np.uint8)   #mask必须行和列都加2，且必须为uint8单通道阵列
    #为什么要加2可以这么理解：当从0行0列开始泛洪填充扫描时，mask多出来的2可以保证扫描的边界上的像素都会被处理
    # if image[719][560] == 0:
    #     cv2.floodFill(image, mask, (640, 719), 255, (100, 100, 100), (50, 50 ,50), cv2.FLOODFILL_FIXED_RANGE)
    # if image[719][640] == 0:
    #     cv2.floodFill(image, mask, (600, 719), 255, (100, 100, 100), (50, 50, 50), cv2.FLOODFILL_FIXED_RANGE)
    # if image[719][720] == 0:
    #     cv2.floodFill(image, mask, (680, 719), 255, (100, 100, 100), (50, 50, 50), cv2.FLOODFILL_FIXED_RANGE)
    cv2.floodFill(image, mask, seed, 255, (100, 100, 100), (50, 50, 50), cv2.FLOODFILL_FIXED_RANGE)
    image = cv2.bitwise_and(image,road_mask)

    return image

# 转鸟瞰图
@jit
def ply2bev(img,point):
    # img = np.zeros((bev_size, bev_size), np.uint8)
    mid_x,mid_y = int(img.shape[0]/2),int(img.shape[1]/2)
    x,y = int(point[0]),int(point[1])
    img[max(min(x, img.shape[0]-1), 0)][max(min(mid_y+y, img.shape[1]-1), 0)] = 255
    # for point in range(ply_data.shape[0]):
    #     x,y,z = -round(10*float(ply_data[point][0])),round(10*float(ply_data[point][1])),int(float(ply_data[point][2]))
    #     # print(ply_data[point])
    #     # z=1
    #     if (x < -y  and -y < 0 ) or (-y < -x  and -y > 0): # FOV = 90
    #         img[max(min(mid_x+x,bev_size-1),0)][max(min(mid_y+y,bev_size-1),0)] = 255
    # return cv2.flip(img,-1)
    return img
# def isLineSimilar():

    # print(image.shape)
    # image.
    # cv2.imshow("fill_color_demo", image)
    # cv2.waitKey(0)

# @jit
def compute_verse(rgb_x,rgb_y):
    b = (rgb_y - 360)*1.0/36
    x = int(round(b * 0.1 *205*2))
    c = (rgb_x - 640)*1.0/64
    # z = 1
    # y = (rgb_x - y_offset)/y_scale
    # s2 = -y/z
    # z2 = z*z
    # s1 = z2 * (s2*s2+1) / (focal_length*focal_length)
    # x2 = (y*y + z*z) / s1
    # x = math.sqrt(x2)
    # # print('s1',s1)
    try:
        y = c/b
    except ZeroDivisionError:
        y = 100
    # print(x*100,y*100)
    return (x, int(round(y*10*4)))

@jit
def compute(b,a,c):
    y = ((a) * 360) /410 +360
    x = ((b-200) * (y-360) /36 *1.6) +640
    # # dis = math.sqrt(x*x+y*y+z*z)
    # s1 = (y*y + z*z) / (x*x)
    # s2 = y/z
    # a = s1*100/(s2*s2+1)
    # b = math.sqrt(a)
    # c = -s2*b
    return ( round(x), round(y))

def myProject(edge,bev):
    for h in range(160):
        for w in range(1280):
            if edge[719-2 * h][w] == 0:
                continue
            else:
                point = compute_verse(w,719-2 * h)
                ply2bev(bev,point)
    return bev

def findF_End(img, x, y, w, h):
    for i in range(h - 1):
        for j in range(w - 1):
            if img[y + i][x + j] == 255:
                f_end = (x + j, y + i)
                return f_end
    return (0, 0)

def findB_End(img, x, y, w, h):
    for i in range(h - 1):
        for j in range(w - 1):
            if img[y + h - i - 1][x + w - j - 1] == 255:
                b_end = (x + w - j - 1, y + h - i - 1)
                return b_end
    return (0, 0)

def findEndPoint(img, x, y, w, h):
    f_end = findF_End(img, x, y, w, h)
    b_end = findB_End(img, x, y, w, h)
    return f_end,b_end

def _lineLength(line):
    point1 = line[0]
    point2 = line[1]
    return pow((line[0][0]-line[1][0]),2) + pow((line[0][1]-line[1][1]),2)
def findLongestLine(lines,part):
    maxLength = 0
    maxIndex = -1
    for index,line in enumerate(lines):
        if part == 0: #左侧
            # if line[0][0] + line[1][0] < 400:
            if line[1][0] < 200:
                length = _lineLength(line)
                if length > maxLength:
                    maxLength = length
                    maxIndex = index
            else:
                continue
        elif part == 1: #右侧
            # if line[0][0] + line[1][0] > 400:
            if line[1][0] > 200:
                length = _lineLength(line)
                if length > maxLength:
                    maxLength = length
                    maxIndex = index
            else:
                continue

    # if maxIndex == -1:
    #     print("Can't find suitable line")
    return maxIndex

def lineExpand_vertical(line,exten_point1 = False):
    point1 = line[0]
    point2 = line[1]
    x,y = point2[0],point2[1]
    # if y < 2 * x and x < 200:
    #     k = (point2[0] - point1[0])/(point2[1] - point1[1])
    #     b = point1[0] - k * point1[1]
    #     x_end = int(k * (2 * x) + b)
    #     y_end = (2 * x)
    #     # print(point1,point2,(x_end,y_end))
    #     return [point1, (x_end,y_end)]
    # elif y < 800 - 2 * x and x > 200:
    #     k = (point2[0] - point1[0]) / (point2[1] - point1[1])
    #     b = point1[0] - k * point1[1]
    #     x_end = int(k * (800 - 2 * x) + b)
    #     y_end = (800 - 2 * x)
    #     # print(point1,point2,(x_end,y_end))
    #     return [point1, (x_end, y_end)]
    # else:
    #     return [point1, point2]
    try:
        k = (point2[0] - point1[0]) / (point2[1] - point1[1])
    except ZeroDivisionError:
        k = 100000
        print('ZeroDivisionError')
    b = point1[0] - k * point1[1]
    x_end = int(k * 719 + b)
    y_end = 719

    if x_end < 0:
        y_end = int(-b / k)
        x_end = 0
    elif x_end > 1279:
        y_end = int((1279-b) / k)
        x_end = 1279
    else:
        y_end = 719

    if exten_point1:
        x_up = int(k * 360 + b)
        y_up = 360
    else:
        x_up = point1[0]
        y_up = point1[1]
    return [(x_up,y_up), (x_end, y_end)]



def lineExpand_horizonal(line,direction,scale):
    point1 = line[0]
    point2 = line[1]
    if direction == 0: #左
        # k = (point1[0]-point2[0])/(point1[1]-point2[1])
        return (point1[0]+scale*100,point1[1]), (point2[0]+scale*100,point2[1])
    elif direction == 1: #右
        # k = (point1[0]-point2[0])/(point1[1]-point2[1])
        return (point1[0]-scale*100,point1[1]), (point2[0]-scale*100,point2[1])

def drawDriveableArea(cur_ll,lines):
    # print(lines)
    if len(lines) == 0 or len(lines) > 8:
        # print("All")
        return (0,0),(399,0),(399,399),(0,399)
    elif cur_ll == -1:
        # print("Not in Driving Area!")
        return (0, 0), (0, 0), (0, 0), (0, 0)
    elif cur_ll == 0:
        l_line_index = findLongestLine(lines, 0)
        # print(l_line_index)
        r_line_index = findLongestLine(lines, 1)
        # print(r_line_index)
        l_line = lines[l_line_index]
        # l_line = lineExpand_vertical(l_line)
        r_line = lines[r_line_index]
        # r_line = lineExpand_vertical(r_line)
        if l_line_index == r_line_index == -1:
            print("Can't change laneline  but can't found laneline!")
            # raise RuntimeError
            return (0, 0), (0, 0), (0, 0), (0, 0)
        elif l_line_index == -1:
            expaned_point1, expaned_point2 = lineExpand_horizonal(r_line, 1, 1)
            return r_line[0], expaned_point1, expaned_point2, r_line[1]
        elif r_line_index == -1:
            expaned_point1, expaned_point2 = lineExpand_horizonal(l_line, 0, 1)
            return l_line[0], expaned_point1, expaned_point2, l_line[1]
        return l_line[0],r_line[0],r_line[1],l_line[1]
    elif cur_ll == 1:
        l_line_index = findLongestLine(lines, 0)
        l_line = lines[l_line_index]
        # l_line = lineExpand_vertical(l_line)
        expaned_point1, expaned_point2 = lineExpand_horizonal(l_line, 0, 2)
        return l_line[0],expaned_point1,expaned_point2,l_line[1]
    elif cur_ll == 2:
        r_line_index = findLongestLine(lines, 1)
        r_line = lines[r_line_index]
        # r_line = lineExpand_vertical(r_line)
        expaned_point1, expaned_point2 = lineExpand_horizonal(r_line, 1, 2)
        return r_line[0],expaned_point1,expaned_point2,r_line[1]
    elif cur_ll == 3:
        l_line_index = findLongestLine(lines, 0)
        r_line_index = findLongestLine(lines, 1)
        l_line = lines[l_line_index]
        # l_line = lineExpand_vertical(l_line)
        r_line = lines[r_line_index]
        # r_line = lineExpand_vertical(r_line)
        expaned_point1, expaned_point2 = lineExpand_horizonal(l_line, 1, 1)
        expaned_point3, expaned_point4 = lineExpand_horizonal(r_line, 0, 1)
        return expaned_point4, expaned_point1, expaned_point2, expaned_point3

# sem = []
# for filename in (glob.glob(semantic_path + '/*.jpg')):
#     sem.append(filename)
#
# for sem_file in tqdm(sem):
#     file = sem_file.strip(semantic_path)
#     ll_sem = file.strip('.jpg')
#     cur_ll = parseLLtxt(ll_sem)
#
#     img = cv2.imread(sem_file)
#     hsv=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
#     mask = cv2.inRange(hsv, lowerb=RoadLine_low_hsv, upperb=RoadLine_high_hsv)
#     road_mask = cv2.inRange(hsv, lowerb=Road_low_hsv, upperb=Road_high_hsv)
#     road_mask = cv2.bitwise_or(road_mask,mask)
#
#     rect = np.array([[0, 360], [0, 400], [1279, 400], [1279, 360]])
#     cv2.fillConvexPoly(mask, rect, 0)
#     # cv2.imshow("img", mask)
#     # cv2.waitKey(0)
#     # img = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
#     # cv2.imwrite(LaneLine_path + '/' + file,mask)
#
#     prewitt1 = np.array([[1, 0, -1]])
#     prewitt2 = np.array([[-1, 0, 1]])
#
#     edge1 = cv2.filter2D(mask,-1,prewitt1)
#     edge2 = cv2.filter2D(mask,-1,prewitt2)
#     edge = cv2.bitwise_or(edge1,edge2)
#     # edge = cv2.Canny(mask,0,100)
#     cv2.imshow("img", edge)
#     cv2.waitKey(0)
#
#     bev = np.zeros((400, 400), np.uint8)
#     bev = myProject(edge,bev)
#     cv2.imshow("img", bev)
#     cv2.waitKey(0)
#     kernel = np.ones((3, 3), np.uint8)
#     bev = cv2.dilate(bev, kernel)
#     # kernel = np.ones((10, 1), np.uint8)
#     # kernel = np.ones((5, 5), np.uint8)
#     bev = cv2.erode(bev, kernel)
#     bev = cv2.medianBlur(bev, 5)
#     bev = cv2.medianBlur(bev, 5)
#
#     # bev = cv2.dilate(bev, kernel)
#
#     bev = cv2.filter2D(bev,-1,prewitt2)
#     # edge2 = cv2.filter2D(bev, -1, prewitt2)
#     # bev = cv2.bitwise_or(edge1, edge2)
#
#
#     cv2.imshow("img", bev)
#     cv2.waitKey(0)
#
#     lines = []
#     bev_lines = []
#     contours, hierarchy = cv2.findContours(bev, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
#     for c in contours:
#         # 找到边界坐标
#         x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
#         # cv2.rectangle(bev,(x,y),(x+w,y+h),100,2)
#         # cv2.putText(mask, str(abs(p1[1]-p2[1])), (x+int(w/2), y+int(h/2)), cv2.FONT_HERSHEY_COMPLEX, 0.5, 100, 1)
#         # 拟合直线
#         if h  > 20:
#             f_end, b_end = findEndPoint(bev, x, y, w, h)
#             cv2.line(bev, f_end, b_end, 100, 2)
#             bev_lines.append([f_end, b_end])
#             try:
#                 lines.append([compute(f_end[0], f_end[1], -1), compute(b_end[0], b_end[1], -1)])
#             except TypeError:
#                 continue
#
#             # [vx, vy, point_x, point_y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
#             # # if vy == 0:
#             # #     vy = 0.0001
#             # k = vx / vy
#             #
#             # try:
#             #     x1 = int(k * (y - point_y) + point_x)
#             #     x2 = int(k * (y+h - point_y) + point_x)
#             # except OverflowError:
#             #     continue
#             # cv2.line(img, (x1, 360), (x2, 719), _RoadLine, 3)
#             # # cv2.line(bev, (x1, y), (x2, y+h), 100, 2)
#             # # print((x1, y), (x2, y+h))
#             # lines.append([compute(x1, y, -1), compute(x2, y + h, -1)])
#             # if x + w / 2 < 200:
#             #     cv2.line(bev, (x + w, y), (x, y + h), 100, 2)
#             #     lines.append([compute(x+w, y, -1), compute(x, y + h, -1)])
#             # else:
#             #     cv2.line(bev, (x, y), (x + w, y + h), 100, 2)
#             #     lines.append([compute(x + w, y+h, -1), compute(x, y, -1)])
#
#     for line in lines:
#         point1 = line[0]
#         point2 = line[1]
#         # print(point1,point2)
#         cv2.line(mask, point1, point2, 100, 2)
#
#     cv2.imshow("img", bev)
#     cv2.waitKey(0)
#     cv2.imshow("img", mask)
#     cv2.waitKey(0)
#
#     p1,p2,p3,p4 = drawDriveableArea(cur_ll,bev_lines)
#     cv2.line(bev, p1, p2, 170, 1)
#     cv2.line(bev, p2, p3, 170, 1)
#     cv2.line(bev, p3, p4, 170, 1)
#     cv2.line(bev, p4, p1, 170, 1)
#     p1, p2, p3, p4 = compute(p1[0],p1[1],-1),compute(p2[0],p2[1],-1),compute(p3[0],p3[1],-1),compute(p4[0],p4[1],-1)
#     cv2.line(mask, p1, p2, 170, 1)
#     cv2.line(mask, p2, p3, 170, 1)
#     cv2.line(mask, p3, p4, 170, 1)
#     cv2.line(mask, p4, p1, 170, 1)
#     cv2.imshow("img", bev)
#     cv2.waitKey(0)
#     cv2.imshow("img", mask)
#     cv2.waitKey(0)




    # for i in range(72):
    #     line_y = 360 + i*5
    #     rect = np.array([[0, line_y], [1279, line_y]])
    #     cv2.fillConvexPoly(mask, rect, 0)
    #     # rect = np.array([[0, 450], [0, 451], [1279, 450], [1279, 451]])
    #     # cv2.fillConvexPoly(mask, rect, 0)
    #     # rect = np.array([[0, 400], [0, 401], [1279, 400], [1279, 401]])
    #     # cv2.fillConvexPoly(mask, rect, 0)
    #
    # img = np.zeros((1000, 1000), np.uint8)
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow("img", mask)
    # cv2.waitKey(0)
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    #     if x + w/2 <640:
    #         point1 = compute_verse(x+w, y)
    #         point2 = compute_verse(x, y+h)
    #     else:
    #         point1 = compute_verse(x + w, y +h)
    #         point2 = compute_verse(x, y)
    #     # print(abs(point1[1] - point2[1]))
    #     img = ply2bev(img,point1)
    #     img = ply2bev(img, point2)
    #     if abs(point1[1] - point2[1]) > 20:
    #         rect = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
    #         cv2.fillConvexPoly(mask, rect, 0)
    #
    # # cv2.imshow("img", mask)
    # # cv2.waitKey(0)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
    #
    # kernel = np.ones((4, 4), np.uint8)
    # mask = cv2.dilate(mask, kernel)
    # mask = cv2.medianBlur(mask, 3)
    # mask = cv2.dilate(mask, kernel)
    # # mask = cv2.medianBlur(mask, 3)
    # # cv2.imshow("img", mask)
    # # cv2.waitKey(0)
    #
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # # cv2.imshow("img", mask)
    # # cv2.waitKey(0)
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    #     if x + w / 2 < 640:
    #         point1 = compute_verse(x + w, y)
    #         point2 = compute_verse(x, y + h)
    #     else:
    #         point1 = compute_verse(x + w, y + h)
    #         point2 = compute_verse(x, y)
    #     # print(abs(point1[1] - point2[1]))
    #     if abs(point1[0] - point2[0]) < 100:
    #         rect = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
    #         cv2.fillConvexPoly(mask, rect, 0)
    #
    # cv2.imshow("img", mask)
    # cv2.waitKey(0)
    # # contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # # for c in contours:
    # # # 找到边界坐标
    # #     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
    # #     # if h/((y-360)+1) >0.3:
    # #     #     cv2.rectangle(mask, (x, y), (x + w, y + h), 70, 1)
    # #
    # # # 找面积最小的矩形
    # # #     rect = cv2.minAreaRect(c)
    # # #     # 得到最小矩形的坐标
    # # #     box = cv2.boxPoints(rect)
    # # #     # 标准化坐标到整数
    # # #     box = np.int0(box)
    # # #     # 画出边界
    # # #     cv2.drawContours(mask, [box], 0, 170, 3)
    # #
    # #     #拟合直线
    # #     if h/((y-360)+1) >0.3:
    # #         rows, cols = img.shape[:2]
    # #         # cols = 1280
    # #         [vx, vy, point_x, point_y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
    # #         k = vx/vy
    # #         x1 = int(k * (360 - point_y) + point_x)
    # #         x2 = int(k * (719 - point_y) + point_x)
    # #         cv2.line(mask, (x1, 360), (x2, 719), 170, 3)
    #         # x1 = int(k * (y - point_y) + point_x)
    #         # x2 = int(k * (y + h - point_y) + point_x)
    #         # cv2.line(mask, (x1, y), (x2, y+h), 170, 2)
    #
    # rect = np.array([[0, 360], [1279, 360]])
    # cv2.fillConvexPoly(mask, rect, 128)
    # da = fill_color_demo(mask,road_mask)
    # print(da.shape)
    # cv2.imshow("img", da)
    # cv2.waitKey(0)
    # retval, dst = cv2.threshold(da, 170, 255, cv2.THRESH_BINARY)
    # cv2.imshow("img", dst)
    # cv2.waitKey(0)