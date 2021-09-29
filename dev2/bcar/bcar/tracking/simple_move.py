#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

MAX_SPEED = 0.18

def contain(a, min, max):
    if a < min:
        a = min
    if a > max:
        a = max
    return a
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
def angleSub(a, b):
    '''a, b 范围 -pi 到 pi '''
    diff = a - b
    
    return normalize_angle(diff)

    
def __claclSpeed(point, car):
    cpos = car.currentPos()
    dis = math.sqrt((point.x - cpos[0])*(point.x - cpos[0]) + \
        (point.y - cpos[1])*(point.y - cpos[1])) 
    yaw = math.atan2(point.y-cpos[1], point.x-cpos[0])
    dyaw = angleSub(cpos[2], yaw)
    if abs(dyaw)>0.35: #20°
        return 0, 0, -0.8*dyaw
    va = -0.8 * dyaw
    if abs(dis) > 0.25:
        return MAX_SPEED, 0, va
    vx = 0.8*dis
    return vx, 0, va
     
def goWithPathEx(paths, car):
    ps = [paths[0].path[0]]
    for path in paths:
        for i in range(1, len(path.path)):
            ps.append(path.path[i])
    skipPoint = [True]
    
    for i in range(1, len(ps)-1):
        a1 = math.atan2(ps[i].y-ps[i-1].y, ps[i].x - ps[i-1].x)
        a2 = math.atan2(ps[i+1].y-ps[i].y, ps[i+1].x-ps[i].x)
        dis = math.sqrt((ps[i].x - ps[i-1].x)*(ps[i].x - ps[i-1].x) + \
            (ps[i].y - ps[i-1].y)*(ps[i].y - ps[i-1].y))
        
        if abs(a1-a2)<0.2 or dis < 0.2:
            skipPoint.append(True)
        else:
            skipPoint.append(False)
    skipPoint.append(False)

    skipIdx = 0
    for path in paths:
        points_number = len(path.path)
        for i in range(points_number):
            if path != paths[0] and i == 0:
                continue
            if skipPoint[skipIdx]:
                skipIdx += 1
                continue
            skipIdx += 1
            
            point = path.path[i]
            print point.tag()
            cpos = car.currentPos()
            
            dis = math.sqrt((point.x - cpos[0])*(point.x - cpos[0]) + \
                (point.y - cpos[1])*(point.y - cpos[1])) 
            yaw = math.atan2(point.y-cpos[1], point.x-cpos[0])
            dyaw = angleSub(cpos[2], yaw)
            while not (abs(dis)<0.07):
                vx, vy, va = __claclSpeed(point, car)
                yield vx, vy, va, path, i
                cpos = car.currentPos()
                
                dis = math.sqrt((point.x - cpos[0])*(point.x - cpos[0]) + \
                    (point.y - cpos[1])*(point.y - cpos[1])) 
                yaw = math.atan2(point.y-cpos[1], point.x-cpos[0])
                dyaw = angleSub(cpos[2], yaw)
    
        
  
