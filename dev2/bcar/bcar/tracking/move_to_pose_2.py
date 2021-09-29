#!/usr/bin/python
# -*- coding: utf-8 -*-
"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai(@Atsushi_twi)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

#import matplotlib.pyplot as plt
import numpy as np
from random import random
import time
import math

 
 
MAX_SPEED = 0.15 #最大限速
MAX_SPEED_YAW = 0.8 #最大旋转限速

def contain(a, min, max):
    if a < min:
        a = min
    if a > max:
        a = max
    return a
def angleSub(a, b):
    '''a, b 范围 -pi 到 pi '''
    diff = a - b
    if diff > math.pi:
        diff = diff - math.pi*2
    elif diff < -math.pi:
        diff = diff + math.pi*2
    return diff
def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def goWithPathEx(paths, car):
    Lfc = 0.2  # 前视距离
    Kp = 1  # 速度P控制器系数
    KAVP = 0.1  # 转向时速度抑制
    KAP = 1.2  # 角速度控制器系数
    KAD = 0.5   # 角速度控制器系数
    lastYaw = 0
    lastErrYaw = 0
 
    ip = 0
    idx_i = 0
    idx_j = 0
    speed_vx = 0
    while idx_i < len(paths):
        while idx_j < len(paths[idx_i].path):
            cx, cy, ca = car.currentPos()
            '''
            tpos = paths[idx_i].path[idx_j]
            dx = cx - tpos.x
            dy = cy - tpos.y
            L = abs(math.sqrt(dx ** 2 + dy ** 2))
            '''
            LS = []
            minL = 100000
            minK = -1
            for k in range(len(paths[idx_i].path)):
                tpos = paths[idx_i].path[k]
                dx = cx - tpos.x
                dy = cy - tpos.y
                L = abs(math.sqrt(dx ** 2 + dy ** 2))
                LS.append(L)
                if L < minL:
                    minL = L
                    minK = k
            print minL, idx_j, minK 
            L = minL
            idx_j = minK
            tpos  = paths[idx_i].path[minK]
            
            K = 1  # 前视距离系数
            Lf = K * speed_vx + Lfc
            ind = 0
            while L < Lf:
                idx_j += 1
                if idx_j >= len(paths[idx_i].path):
                    idx_i += 1
                    idx_j = 0
                    if idx_i >= len(paths):
                        idx_i -= 1
                        idx_j = len(paths[idx_i].path)-1
                        break
                npos = paths[idx_i].path[idx_j]
                dx = npos.x - tpos.x
                dy = npos.y - tpos.y
                L += math.sqrt(dx ** 2 + dy ** 2)
                tpos = npos
                
            target_speed = Lf * KAP;
            if L < Lfc :  
                #finish 
                return
                
            #当前距离大于前视距离，加速
            speed_vx += Kp * (target_speed - speed_vx)

            
            alpha = math.atan2(tpos.y - cy, tpos.x - cx)
            alpha = angleSub(alpha, ca)
            if abs(alpha) > math.pi / 4:
                speed_vx = 0
              
            elif abs(alpha) > math.pi/180*10 and speed_vx == 0:
                pass
            else:
                speed_vx -= abs(alpha * KAVP)
                if speed_vx < 0:
                    speed_vx = 0
             

            speed_ca = KAP * alpha + (alpha - lastYaw)*KAD
            lastYaw = alpha
            speed_vx = contain(speed_vx, -MAX_SPEED, MAX_SPEED)
            speed_ca = contain(speed_ca, -MAX_SPEED_YAW, MAX_SPEED_YAW)
            yield speed_vx, 0, speed_ca, paths[idx_i], idx_j
            
if __name__ == '__main__':
    pass
