#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

# Parameters
 #Lf = k * state.v + Lfc
k = 1       #前视距离增益
Lfc = 0.22  # 前视距离
L = 0.22
KV = 1


MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/2

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
    
def goWithPath(paths, car):
    sizep = 0
    for path in paths:
        for p in path.path:
            sizep += 1
    ip = 0
    idx_i = idx_j = 0
    dis = 0
    while ip < sizep - 1 or dis >= 0.1:
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

        va = pure_pursuit_control(car, tpos)
        v = MAX_SPEED - abs(va * 0.18)

        car.updateSpeed(v, va)
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
    