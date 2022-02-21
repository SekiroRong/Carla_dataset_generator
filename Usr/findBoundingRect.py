# -*- coding = utf-8 -*-
# @Time : 2021/12/12 13:14
# @Author : 戎昱
# @File : findBoundingRect.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import cv2
import numpy as np

from config import status

semantic_path = r"G:\Carla_Recorder\semantic_Recorder" + '/' + status

def findVehicleBoundingRect_plus(sem_img):
    # 图像转灰度图
    sem_img = cv2.cvtColor(sem_img, cv2.COLOR_BGR2GRAY)
    # 图像转二值图
    ret, thresh = cv2.threshold(sem_img, 17, 255, cv2.THRESH_BINARY_INV)
    # 滤波
    kernel = np.ones((0, 0), np.uint8)
    blur = cv2.dilate(thresh, kernel)

    # 功能：cv2.findContours()函数来查找检测物体的轮廓。
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for c in contours:
        # 找到边界坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        rects.append([x, y,x + w, y + h])

    return rects

def findWalkerBoundingRect_plus(sem_img):
    # 图像转灰度图
    # print(sem_img.shape)
    sem_img = cv2.cvtColor(sem_img, cv2.COLOR_BGR2GRAY)
    # 图像转二值图
    ret, thresh = cv2.threshold(sem_img, 83, 255, cv2.THRESH_BINARY_INV)
    ret2, thresh2 = cv2.threshold(sem_img, 85, 255, cv2.THRESH_BINARY_INV)
    thresh3 = cv2.bitwise_xor(thresh, thresh2)
    # 滤波
    kernel = np.ones((0, 0), np.uint8)
    blur = cv2.dilate(thresh3, kernel)

    # 功能：cv2.findContours()函数来查找检测物体的轮廓。
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for c in contours:
        # 找到边界坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        rects.append([x, y,x + w, y + h])

    return rects

def findVehicleBoundingRect(sem_img):
    # 图像转灰度图
    sem_img = cv2.cvtColor(sem_img, cv2.COLOR_BGR2GRAY)
    # 图像转二值图
    ret, thresh = cv2.threshold(sem_img, 17, 255, cv2.THRESH_BINARY_INV)
    # 滤波
    blur = cv2.medianBlur(thresh, 5)
    blur = cv2.medianBlur(blur, 5)
    blur = cv2.medianBlur(blur, 5)
    blur = cv2.medianBlur(blur, 5)

    # 功能：cv2.findContours()函数来查找检测物体的轮廓。
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for c in contours:
        # 找到边界坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        rects.append([(x, y),(x + w, y + h)])

    # l = len(rects)
    # if l == 0 or l ==1:
    #     return rects
    # else:
    #     # print(l)
    #     for i in range(l):
    #         for j in range(l-i):
    #             # print(rects[i],rects[i+j])
    #             try:
    #                 result = _isIndside(rects[i],rects[i+j])
    #                 if result == 1:
    #                     rects.pop(i+j)
    #                     # print(len(rects))
    #                     # i -= 1
    #                     print("Absorbed")
    #                 elif result == -1:
    #                     rects[i].remove()
    #                     # rects[i] = rects[i+j]
    #                     print("beAbsorbed")
    #             except IndexError:
    #                 do = 1
    #                 # print('')

    # rects = (set(rects))
    # print(rects)
    return rects

def _isIndside(rect1,rect2):
    if rect1[0][0] <= rect2[0][0] and rect1[0][1] <= rect2[0][1] and rect1[1][0] >= rect2[1][0] and rect1[1][1] >= rect2[1][1]:
        # 1包2
        return 1
    elif rect1[0][0] >= rect2[0][0] and rect1[0][1] >= rect2[0][1] and rect1[1][0] <= rect2[1][0] and rect1[1][1] <= rect2[1][1]:
        # 2包1
        return -1
    else:
        return 0

def iou_2d_plus(points1, points2):
    '''
        box [x1,y1,x2,y2]   分别是两对角定点的坐标
    '''
    # box1 = [min(points1[0][0],points1[1][0],points1[2][0],points1[3][0]),
    #         min(points1[4][1],points1[5][1],points1[6][1],points1[7][1]),
    #         max(points1[4][0],points1[5][0],points1[6][0],points1[7][0]),
    #         max(points1[0][1],points1[1][1],points1[2][1],points1[4][1])]
    #
    # box2 = [min(points2[0][0],points2[1][0],points2[2][0],points2[3][0]),
    #         min(points2[4][1],points2[5][1],points2[6][1],points2[7][1]),
    #         max(points2[4][0],points2[5][0],points2[6][0],points2[7][0]),
    #         max(points2[0][1],points2[1][1],points2[2][1],points2[3][1])]
    box1 = points1
    box2 = points2

    # print(points1)
    # print(points2)
    #
    # print(box1)
    # print(box2)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    area_sum = area1 + area2
    small_area = min(area1,area2)
    scale = 1
    if area1 > area2:
        scale = -1
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

def findWalkerBoundingRect(sem_img):
    # 图像转灰度图
    sem_img = cv2.cvtColor(sem_img, cv2.COLOR_BGR2GRAY)
    # 图像转二值图
    ret, thresh = cv2.threshold(sem_img, 83, 255, cv2.THRESH_BINARY_INV)
    ret2, thresh2 = cv2.threshold(sem_img, 85, 255, cv2.THRESH_BINARY_INV)
    thresh3 = cv2.bitwise_xor(thresh, thresh2)
    # 滤波
    blur = cv2.medianBlur(thresh3, 5)
    blur = cv2.medianBlur(blur, 5)
    blur = cv2.medianBlur(blur, 5)
    blur = cv2.medianBlur(blur, 5)

    # 功能：cv2.findContours()函数来查找检测物体的轮廓。
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for c in contours:
        # 找到边界坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        rects.append([(x, y),(x + w, y + h)])

    return rects


# sem = semantic_path + '/' + '0_000590.jpg'
# image = cv2.imread(sem)
# # 图像转灰度图
# img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imshow("img", img)
# # cv2.imwrite("img_1.jpg", image)
# cv2.waitKey(0)
# print(img[360][640])
#
# # 图像转二值图
# ret, thresh = cv2.threshold(img, 83, 255, cv2.THRESH_BINARY_INV)
# ret2, thresh2 = cv2.threshold(img, 85, 255, cv2.THRESH_BINARY_INV)

# cv2.imshow("img", thresh)
# cv2.waitKey(0)
#
# cv2.imshow("img", thresh2)
# cv2.waitKey(0)
#
# thresh3 = cv2.bitwise_xor(thresh,thresh2)
# cv2.imshow("img", thresh3)
# cv2.waitKey(0)
#
# blur = cv2.medianBlur(thresh3,5)
# blur = cv2.medianBlur(blur,5)
# blur = cv2.medianBlur(blur,5)
# blur = cv2.medianBlur(blur,5)
# cv2.imshow("img", blur)
# cv2.waitKey(0)
# # 功能：cv2.findContours()函数来查找检测物体的轮廓。
# #参数:
# # 参数1：寻找轮廓的图像，接收的参数为二值图，即黑白的（不是灰度图），所以读取的图像要先转成灰度的，再转成二值图
# # 参数2: 轮廓的检索模式，有四种。
# #       cv2.RETR_EXTERNAL 表示只检测外轮廓;
# #       cv2.RETR_LIST 检测的轮廓不建立等级关系;
# #       cv2.RETR_CCOMP 建立两个等级的轮廓，上面的一层为外边界，里面的一层为内孔的边界信息。如果内孔内还有一个连通物体，这个物体的边界也在顶层。
# #       cv2.RETR_TREE 建立一个等级树结构的轮廓。
# #
# # 参数3: 轮廓的近似办法.
# #       cv2.CHAIN_APPROX_NONE 存储所有的轮廓点，相邻的两个点的像素位置差不超过1，即max（abs（x1-x2），abs（y2-y1））==1
# #       cv2.CHAIN_APPROX_SIMPLE 压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标，例如一个矩形轮廓只需4个点来保存轮廓信息
# #       cv2.CHAIN_APPROX_TC89_L1，CV_CHAIN_APPROX_TC89_KCOS 使用teh-Chinl chain 近似算法
# # 注：opencv2返回两个值：contours：hierarchy。opencv3会返回三个值,分别是img, countours, hierarchy
# #
# #返回值
# #cv2.findContours()函数返回两个值，一个是轮廓本身，还有一个是每条轮廓对应的属性。
# contours, hierarchy = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
# for c in contours:
#     # 找到边界坐标
#     x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
#     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#
#     # # 找面积最小的矩形
#     # rect = cv2.minAreaRect(c)
#     # # 得到最小矩形的坐标
#     # box = cv2.boxPoints(rect)
#     # # 标准化坐标到整数
#     # box = np.int0(box)
#     # 画出边界
#     # cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
#     # # 计算最小封闭圆的中心和半径
#     # (x, y), radius = cv2.minEnclosingCircle(c)
#     # # 换成整数integer
#     # center = (int(x), int(y))
#     # radius = int(radius)
#     # # 画圆
#     # cv2.circle(image, center, radius, (0, 255, 0), 2)
#
# # cv2.drawContours(image, contours, -1, (255, 0, 0), 1)
# cv2.imshow("img", image)
# # cv2.imwrite("img_1.jpg", image)
# cv2.waitKey(0)