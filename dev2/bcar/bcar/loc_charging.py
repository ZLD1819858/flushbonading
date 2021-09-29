#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from copy import deepcopy
import math
import threading
import numpy as np
import sys
import time
import tf
import os
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2


class LocCharging:
    
    def __init__(self):
        self.laserProj = LaserProjection()
        self.__scan = rospy.Subscriber('/scan_f', LaserScan, self.__scanCallback)
        self.baseLinkTrans = None
        #self.__miniDis = (float('inf'),0)
        self.__distance = None
        self.lastLimitTime = 0
        self.__pointcloud = None
        
    def getDistance(self, ang):
        while ang >= math.pi*2:
            ang -= math.pi*2
        while ang < 0:
            ang += math.pi*2
        if self.__distance == None:
            return float('inf')
        msg = self.__distance
        ii = int(ang / msg.angle_increment+0.5)%len(msg.ranges)
        
        return msg.ranges[ii]
        
    def findMinAngDis(self, st, end):
        if self.__distance == None:
            return float('inf'), float('inf')
         
        angs = []
        while st <= end :
            dis = self.getDistance(st)
            angs.append((st, dis))
            st += self.__distance.angle_increment
        mina = float('inf')
        mind = float('inf')
        for a,d in angs:
            if d < mind:
                mina = a
                mind = d
        return mina, mind
                
    def __scanCallback(self, msg):
 
        if self.baseLinkTrans == None:
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform('/base_link', msg.header.frame_id, rospy.Time(), rospy.Duration(1))
            (trans, rot) = tf_listener.lookupTransform('/base_link', msg.header.frame_id, rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            self.baseLinkTrans = (trans, euler)  # 雷达坐标到baselink转换
            #print self.baseLinkTrans
        #旋转雷达坐标到base link
        idx = int(self.baseLinkTrans[1][2]/msg.angle_increment)
        msg.ranges =  msg.ranges[-idx:]+msg.ranges[0:-idx]
        self.__distance = msg
    
    def __del__(self):
        self.__scan.unregister()
        
        
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
    
    
__pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=0, latch=False)
loc = LocCharging()

def updateSpeed(vx, va, vy):
    t = Twist()
    t.linear.x = vx
    t.linear.y = vy
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = va
    __pubVel.publish(t)
    
MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/4
def applyAngle(car, angle):
    cx, cy, ca = car.currentPos()
    da = normalize_angle(angle - ca) #目标点航向与小车当前航向夹角
    while abs(da) > 0.05:
        va = da * 2.1
        if va > MAX_SPEED_A:
            va = MAX_SPEED_A
        elif va < -MAX_SPEED_A:
            va = -MAX_SPEED_A
        car.updateSpeed(0, va, 0)
        time.sleep(0.1)
        cx, cy, ca = car.currentPos()
        da = normalize_angle(a - ca)
    car.updateSpeed(0, 0, 0)
    
def applyLoc(angle=225*math.pi/180, ddis=0.42):
    '''135度角对齐路灯杆，当角度>135度，小车右移，
        <135度，小车左移'''
    #loc = LocCharging()
    
    angRange = [angle-math.pi/8, angle+math.pi/8]
    ang, dis = loc.findMinAngDis(angRange[0], angRange[1])
    da = normalize_angle(ang - angle)
    
    while abs(da) > 0.05:
        vy = -1 * da * 10
        if vy > 0.15:
            vy = 0.15
        if vy < -0.15:
            vy = -0.15
        print "ang", "%.1f"%(ang*180/math.pi), "%.2f"%(da*180/math.pi), vy
        updateSpeed(0, 0, vy)
        time.sleep(0.1)
        ang, dis = loc.findMinAngDis(angRange[0], angRange[1])
        da = normalize_angle(ang - angle)
    updateSpeed(0, 0, 0)
    dd = dis - ddis
    while abs(dd) > 0.01:
        if dd < 0:
            vx = vy = 0.15
        elif dd > 0:
            vx = vy = -0.15
        print 'dis', "%.2f"%dd, "%.2f"%vy
        updateSpeed(vx, 0, vy)
        time.sleep(0.1)
        angle, dis = loc.findMinAngDis(angRange[0], angRange[1])
        dd = dis - ddis
    updateSpeed(0, 0, 0)
    
    angle, dis = loc.findMinAngDis(angRange[0], angRange[1])
    aa = angle*180/math.pi
    print 'finish', "%.1f"%aa, "%.3f"%dis
    
def locPox(car, angle, o_angle=225*math.pi/180, o_dis=0.42):
    applyAngle(car, angle)
    applyLoc(o_angle, o_dis)
    applyAngle(car, angle)
    
        
        
if __name__ == '__main__':
    import rospy
    import sys
    from car import xcar
    
    rospy.init_node("radar-demo", log_level=rospy.INFO)
    
    car = xcar.ACar()
    
    ra = 0
    ckrange=[202, 247]
    if len(sys.argv) == 3:
        ckrange[0] = int(sys.argv[1])
        ckrange[1] = int(sys.argv[2])
    t = time.time()
    while   time.time() - t < 10:
        for i in range(ckrange[0], ckrange[1], 1):
            print i, '-->', "%.2f"%loc.getDistance(i*math.pi/180)
        print ''
        a,d = loc.findMinAngDis(ckrange[0]*math.pi/180, ckrange[1]*math.pi/180)
        if a == float('inf'):
            a = ra
        aa = a*180/math.pi
        ra = ra*0.8 + aa * 0.2
        print "%.0f"%ra, d
        print '=================================================='
        time.sleep(0.2)
    x,y, a = car.currentPos()
    locPox(car, a)
