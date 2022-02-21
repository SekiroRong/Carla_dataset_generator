# -*- coding = utf-8 -*-
# @Time : 2022/1/3 20:09
# @Author : 戎昱
# @File : depth_cam_demo.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import cv2
import numpy as np
from config import status

depth_path = r"G:\Carla_Recorder\Depth_Recorder" + '/' + status

# img_path = depth_path + '/' + '0_006090.jpg'
#
# img = cv2.imread(img_path)
#
# resImg = cv2.Laplacian(img, -1)
# gray_lap = cv2.Laplacian(img,cv2.CV_16S,ksize = 3)
# # canny = cv2.Canny(img, 50, 150)
#
# prewitt1 = np.array([[1, 0, -1]])
# prewitt2 = np.array([[-1, 0, 1]])
#
# edge1 = cv2.filter2D(img, -1, prewitt1)
# edge2 = cv2.filter2D(img, -1, prewitt2)
# edge = cv2.bitwise_or(edge1, edge2)
#
# retval,edge = cv2.threshold(edge, 10, 255, cv2.THRESH_BINARY)
#
# cv2.imshow("canny", edge)
# cv2.waitKey(0)

def parse_depth_img(path):
    depth_img = depth_path + '/' + path
    img = cv2.imread(depth_img)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    prewitt1 = np.array([[1, 0, -1]])
    prewitt2 = np.array([[-1, 0, 1]])

    edge1 = cv2.filter2D(img, -1, prewitt1)
    edge2 = cv2.filter2D(img, -1, prewitt2)
    edge = cv2.bitwise_or(edge1, edge2)

    retval, img = cv2.threshold(edge, 2, 255, cv2.THRESH_BINARY)

    return img

def from_depth_get_bbox(img,bbox): # bbox:xywh
    # print(img.shape)
    # print(img)
    # depth_img = depth_path + '/' + path
    # img = cv2.imread(depth_img)
    #
    # prewitt1 = np.array([[1, 0, -1]])
    # prewitt2 = np.array([[-1, 0, 1]])
    #
    # edge1 = cv2.filter2D(img, -1, prewitt1)
    # edge2 = cv2.filter2D(img, -1, prewitt2)
    # edge = cv2.bitwise_or(edge1, edge2)
    #
    # retval, img = cv2.threshold(edge, 10, 255, cv2.THRESH_BINARY)
    pos1 = 0.25
    pos2 = 0.5
    pos3 = 0.75

    y1 = int((pos1 * bbox[1] + (1 - pos1) * bbox[3]))
    y2 = int((pos2 * bbox[1] + (1 - pos2) * bbox[3]))
    y3 = int((pos3 * bbox[1] + (1 - pos3) * bbox[3]))
    x_l = bbox[0]
    x_r = bbox[2]
    w = x_r - x_l
    # print(bbox)
    # print(w)
    if not x_l == 0:
        for i in range(w):
            if img[y1][x_l+i] or img[y2][x_l+i] or img[y3][x_l+i]:
                x_l = x_l+i
                break
            elif i == w-1:
                return [0,0,0,0]

    if not x_r == 1279:
        for i in range(w):
            if img[y1][x_r-i] or img[y2][x_r-i] or img[y3][x_r-i]:
                x_r = x_r-i
                break
            elif i == bbox[2]-1:
                return [0,0,0,0]

    return [x_l,bbox[1],x_r,bbox[3]]

