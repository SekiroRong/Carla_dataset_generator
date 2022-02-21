# -*- coding = utf-8 -*-
# @Time : 2021/12/22 10:09
# @Author : 戎昱
# @File : parseLLtxt.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
from config import status

laneline_file = r"G:\Carla_Recorder\Laneline_Recorder" + '/' + status
# carla.LaneChange
# NONE
# Traffic rules do not allow turning right or left, only going straight.
# Right
# Traffic rules allow turning right.
# Left
# Traffic rules allow turning left.
# Both
# Traffic rules allow turning either right or left.
def parseLLtxt(path):
    txt_path = laneline_file + '/' + path + '.txt'
    with open(txt_path,'r') as f:
        line = f.readline()
        content = line.split(' ')
        # print(len(content))
        m_type, m_change = content[0], content[1]
        l_type, l_change = content[2], content[3]
        r_type, r_change = content[4], content[5]
        if m_type == 'Driving':
            if m_change == 'NONE':
                # print("NONE")
                return 0
            elif m_change == 'Both':
                # print("Both")
                return 3
            elif m_change == 'Right':
                # print("Right")
                return 1
            elif m_change == 'Left':
                # print("Left")
                return 2

        else:
            # print(path + " not in Driving Area!")
            return -1


# print(parseLLtxt('1_000290'))