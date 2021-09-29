#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

# Parameters
k = 0.3 #0.16    #转向增益
L = 0.230 #0.4     #数字越大转向提前


MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/2

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
 
    
def goWithPath(paths, car):
    cyaw = []
    points = []
    p0 = None #paths[0].path[0]
    for path in paths:
        for p in path.path:
            if p0 == None:
                p0 = p
                points.append(p0)
            else:
                yaw = math.atan2(p.y-p0.y, p.x-p0.x)
                cyaw.append(yaw)
                p0 = p
                points.append(p)
    cyaw.append(yaw)
    sizep = len(cyaw)
    endt = time.time() + 60
    ip = 0
    while ip < sizep - 1:
    
        cx, cy, ca = car.currentPos()
        # Calc front axle position
        fx = cx + L * np.cos(ca)
        fy = cy + L * np.sin(ca)

        # Search nearest point index
        dx = [fx - icx.x for icx in points[ip:]]
        dy = [fy - icy.y for icy in points[ip:]]
        d = np.hypot(dx, dy)
        
        target_idx = np.argmin(d)
      
         # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(ca + np.pi / 2), -np.sin(ca + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    
 
        ip += target_idx
       
        # theta_e corrects the heading error
        theta_e = normalize_angle(cyaw[ip] - ca)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, car.speed_x)
        # Steering control
        delta = theta_e + theta_d
        

        v = MAX_SPEED #- delta * 0.08
        #w = delta * v / L
       
        car.updateSpeed(v, delta)
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
    