#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

# Parameters
 #Lf = k * state.v + Lfc
k = 0       #前视距离增益
Lfc = 0.2  # 前视距离
L = 0.22
KV = 1


MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/4

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
def pure_pursuit_control(car, tpos):

    cx, cy, ca = car.currentPos()

    alpha = math.atan2(tpos.y - cy, tpos.x - cx)
    
    alpha = angleSub(alpha, ca)
    
    dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
    
    Lf = k * car.speed_x + Lfc
    '''
    if car.speed_x  > 0:
        t = dis / car.speed_x
        va =  2.5*alpha / t
    else:
        va = alpha;
 
    #va = contain(va, -MAX_SPEED_A, MAX_SPEED_A)
    return va
    '''
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    delta = contain(delta, -MAX_SPEED_A, MAX_SPEED_A)
    
    return delta

def goWithPathEx(paths, car):
    cyaw = []
    p0 = None #paths[0].path[0]
    skip = 0
    for path in paths:
        for p in path.path:
            if p0 == None:
                p0 = p
            else:
                dis = math.sqrt((p.x - p0.x)**2 + (p.y - p0.y)**2)
                if dis >= 0.1: #防止距离太小角度抖动
                    yaw = math.atan2(p.y-p0.y, p.x-p0.x)
                    cyaw.append(yaw)
                    while skip > 0: 
                        cyaw.append(yaw)
                        skip -= 1
                    p0 = p
                else:
                    skip += 1

    cyaw.append(yaw)
    sizep = len(cyaw)

    ip = 0
    idx_i = 0
    idx_j = 0
    while ip < sizep - 1 or dis >= 0.2:
        cx, cy, ca = car.currentPos()
        tpos = paths[idx_i].path[idx_j]
        
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        
        Lf = k * car.speed_x + Lfc
        while dis < Lf and ip < sizep-1:
            ip += 1
            idx_j += 1
            if idx_j >= len(paths[idx_i].path) :
                idx_j = 0
                idx_i += 1
            lastpos = tpos
            tpos = paths[idx_i].path[idx_j]
            
            dis += math.sqrt((tpos.x - lastpos.x)**2 + (tpos.y - lastpos.y)**2)
        
            
        cx, cy, ca = car.currentPos()
        alpha = math.atan2(tpos.y - cy, tpos.x - cx)
        
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        
        de = normalize_angle(alpha-ca)  #小车航向与当前位置到目标点夹角
        bate = normalize_angle(cyaw[ip]- alpha) #目标航向与当前位置到目标点夹角
        
        da = normalize_angle(cyaw[ip]- ca) #目标点航向与小车当前航向夹角
        dw = dis * math.cos(bate) #横向误差
        v = MAX_SPEED
       
        vx = v * math.cos(de)  #小车移向目标点的速度
        vy = v * math.sin(de)
        va = da/(dis / v)
        if va > MAX_SPEED_A:
            va = MAX_SPEED_A
        elif va < -MAX_SPEED_A:
            va = -MAX_SPEED_A
            
        yield vx, vy, va, paths[idx_i], idx_j
        
    
def goWithPath(paths, car):
    cyaw = []
    p0 = None #paths[0].path[0]
    skip = 0
    for path in paths:
        for p in path.path:
            if p0 == None:
                p0 = p
            else:
                dis = math.sqrt((p.x - p0.x)**2 + (p.y - p0.y)**2)
                if dis >= 0.1: #防止距离太小角度抖动
                    yaw = math.atan2(p.y-p0.y, p.x-p0.x)
                    cyaw.append(yaw)
                    while skip > 0: 
                        cyaw.append(yaw)
                        skip -= 1
                    p0 = p
                else:
                    skip += 1

    cyaw.append(yaw)
    sizep = len(cyaw)

    ip = 0
    idx_i = 0
    idx_j = 0
    while ip < sizep - 1 or dis >= 0.2:
        cx, cy, ca = car.currentPos()
        tpos = paths[idx_i].path[idx_j]
        
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        
        Lf = k * car.speed_x + Lfc
        while dis < Lf and ip < sizep-1:
            ip += 1
            idx_j += 1
            if idx_j >= len(paths[idx_i].path) :
                idx_j = 0
                idx_i += 1
            lastpos = tpos
            tpos = paths[idx_i].path[idx_j]
            
            dis += math.sqrt((tpos.x - lastpos.x)**2 + (tpos.y - lastpos.y)**2)
        
            
        cx, cy, ca = car.currentPos()
        alpha = math.atan2(tpos.y - cy, tpos.x - cx)
        
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        
        de = normalize_angle(alpha-ca)  #小车航向与当前位置到目标点夹角
        bate = normalize_angle(cyaw[ip]- alpha) #目标航向与当前位置到目标点夹角
        
        da = normalize_angle(cyaw[ip]- ca) #目标点航向与小车当前航向夹角
        dw = dis * math.cos(bate) #横向误差
        v = MAX_SPEED
       
        vx = v * math.cos(de)  #小车移向目标点的速度
        vy = v * math.sin(de)
        va = da/(dis / v)
        
        car.updateSpeed(vx, va, vy)
        time.sleep(0.1)

    car.updateSpeed(0, 0)

    
if __name__ == '__main__':
    import rospy
    import sys
    import cv2
    from scar import ACar
    from map import map_path
    from map import map
    import copy
    import threading
    
    Parks =  map_path.Parks 
    
    rospy.init_node("xcar-demo", log_level=rospy.INFO)
    car = ACar()
    yaws = [-math.pi/2,-math.pi/2, math.pi/2,math.pi/2]
    
    start = 1
    end = 0
    if len(sys.argv) == 3:
        start = int(sys.argv[1])
        end = int(sys.argv[2])
    car.initPos(Parks[start].x, Parks[start].y, yaws[start])
    path = map_path.getFirstPathBySpan(Parks[start], Parks[end])
    map_path.dumpPath(path)
    path = (map_path.path2cuve(path, 0.6),)
    img = map.getImage()
    map.drawPath(img, path, (0,255,0), 2)
    cv2.waitKey(1)
    
    ps = []
    for pa in path:
        for p in pa.path:
            ps.append(p)
    
    
    t = threading.Thread(target=goWithPath, args=(ps, car))
    t.setDaemon(True)
    t.start()
    
    time.sleep(0.1)
    
    gpath = []
    sp1 = car.currentPos()
    while t.is_alive():
        sp2 = car.currentPos()
        dx = sp2[0] - sp1[0]
        dy = sp2[1] - sp1[1]
        d = math.sqrt(dx*dx + dy*dy)
        if d > 0.01:
            gpath.append(map_path.Point(sp2[0], sp2[1]))
            map.drawPath(img, (map_path.Path(gpath),), (255,0,255), 2)
            #map.drawPoint(img, pos[car.target_idx].point)
            
            cv2.imshow("map", img)
        ret = cv2.waitKey(200)
        if ret == ord('q'):
            break
    del car
    cv2.destroyWindow("map")
    