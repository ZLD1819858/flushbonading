#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

# Parameters
# simulation parameters
Kp_rho = 9
Kp_alpha = 1.5 #15
Kp_beta =  -0.3 #-3

Lfc = 0.15
k = 1

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
 
    
def goWithPath(paths, car):
    cyaw = []
    p0 = None #paths[0].path[0]
    for path in paths:
        for p in path.path:
            if p0 == None:
                p0 = p
            else:
                yaw = math.atan2(p.y-p0.y, p.x-p0.x)
                cyaw.append(yaw)
                p0 = p

    cyaw.append(yaw)
    sizep = len(cyaw)
    
    ip = 0
    idx_i = 0
    idx_j = 0
    print   car.currentPos()
    endt = time.time() + 120
    while True or time.time() - endt < 0:
    
        cx, cy, ca = car.currentPos()
        tpos = paths[idx_i].path[idx_j]
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        
        Lf = k * car.speed_x + Lfc
        while dis < Lf and ip < sizep-1:
            idx_j += 1
            if idx_j >= len(paths[idx_i].path) :
                idx_j = 0
                idx_i += 1
            lastpos = tpos
            tpos = paths[idx_i].path[idx_j]
            
            dis += math.sqrt((tpos.x - lastpos.x)**2 + (tpos.y - lastpos.y)**2)
        if ip >= sizep - 1:
            break

        x_goal = paths[idx_i].path[idx_j].x
        y_goal = paths[idx_i].path[idx_j].y
        theta_goal = cyaw[ip]
        x = cx
        y = cy
        theta = ca
        x_diff = x_goal - x
        y_diff = y_goal - y

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        
        v = np.clip(v, -MAX_SPEED, MAX_SPEED)
        w = np.clip(w, -MAX_SPEED_A, MAX_SPEED_A)
    
        car.updateSpeed(v, w)
        time.sleep(0.1)
        
    car.updateSpeed(0, 0)
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
    