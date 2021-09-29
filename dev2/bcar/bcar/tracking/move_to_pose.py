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

MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/4


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

def move_to_pose(car, x, y, a, ed=0.01, ea=0.05):
    cx, cy, ca = car.currentPos()
    dis = math.sqrt((x - cx)**2 + (y - cy)**2)
    da = normalize_angle(a - ca) #目标点航向与小车当前航向夹角
    while dis >= ed:
        alpha = math.atan2(y - cy, x - cx)
        
        de = normalize_angle(alpha-ca)  #小车航向与当前位置到目标点夹角
        bate = normalize_angle(a - alpha) #目标航向与当前位置到目标点夹角
        da = normalize_angle(a - ca) #目标点航向与小车当前航向夹角
        dw = dis * math.cos(bate) #横向误差
        if dis > MAX_SPEED:
            v = MAX_SPEED
        else:
            v = dis
        vx = v * math.cos(de)  #小车移向目标点的速度
        vy = v * math.sin(de)
        va = da/(dis / v)
        if va > MAX_SPEED_A:
            va = MAX_SPEED_A
        elif va < -MAX_SPEED_A:
            va = -MAX_SPEED_A
        car.updateSpeed(vx, va, vy)
        time.sleep(0.1)
        cx, cy, ca = car.currentPos()
        dis = math.sqrt((x - cx)**2 + (y - cy)**2)
        da = normalize_angle(a - ca) #目标点航向与小车当前航向夹角
    while abs(da) >= ea:
        va = da
        if va > MAX_SPEED_A:
            va = MAX_SPEED_A
        elif va < -MAX_SPEED_A:
            va = -MAX_SPEED_A
        car.updateSpeed(0, va, 0)
        time.sleep(0.1)
        cx, cy, ca = car.currentPos()
        da = normalize_angle(a - ca)
    car.updateSpeed(0, 0, 0)
    
if __name__ == '__main__':
    pass
