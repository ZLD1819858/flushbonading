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
CAR_W = 0.3 #车体宽
CAR_H = 0.4 #车体长
CHK_W = 0.02
CHK_H = 0.02
hanfAngle = math.asin(CAR_W/CAR_H)*180/math.pi # 36.8°
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
    
class PluseRadar:
    
    def __init__(self):
        self.laserProj = LaserProjection()
        self.__scan = rospy.Subscriber('/scan_f', LaserScan, self.__scanCallback)
        self.baseLinkTrans = None
        #self.__miniDis = (float('inf'),0)
        self.__distance = None
        self.lastLimitTime = 0
        self.__pointcloud = None
    
    def waitRadar(self, timeout=10):
        if timeout == 0:
            while self.__pointcloud == None:
                time.sleep(1)
        else:
            while self.__pointcloud == None and timeout != 0:
                time.sleep(1)
                timeout -=1
        return self.__pointcloud != None
        
    def getMiniDistance(self, angle=0):
        '''base_link 指定方向 -hanfAngle°到 + hanfAngle°的距离检测 '''
        angle = normalize_angle(angle);

        minDis = (float('inf'), 0)
        a = int(angle * ( 180 / math.pi)+0.5)
        ang = (-hanfAngle+a, hanfAngle+a)
        for aa in range(ang[0], ang[1]):
            angle = aa * math.pi / 180 - self.baseLinkTrans[1][2]
            rr = self.__getDistance( self.__distance, angle)
            if rr > CAR_H/2 and rr < minDis[0]:
                minDis = (rr, aa)
        return minDis

    def clacMiniDistance(self, dir=0):
        pc = pc2.read_points(self.__pointcloud, skip_nans=True, field_names=("x", "y", "z"))
        v = float('inf')
        if dir == 0 or dir == 1:
            if dir == 0: #正前方
                mind = -CAR_W/2 - self.baseLinkTrans[0][1] + CHK_W #y 轴最小  
                maxd = CAR_W/2  - self.baseLinkTrans[0][1] - CHK_W #y 轴最大
                cidx = 1         #检测坐标y
                vidx = 0         #检测值 x
                vmin = CAR_H/2 - self.baseLinkTrans[0][0] #最小值过滤
            elif dir == 1: #左侧
                mind = -CAR_H/2 - self.baseLinkTrans[0][0] + CHK_H
                maxd = CAR_H/2 - self.baseLinkTrans[0][0] - CHK_H
                cidx = 0         #检测坐标x
                vidx = 1         #检测值 y
                vmin = CAR_W/2 - self.baseLinkTrans[0][1] #最小值过滤

            for p in pc:
                if p[cidx] >= mind and p[cidx] <= maxd:
                    if p[vidx] < vmin-0.05:
                        continue
                    if p[vidx] < v:
                        v = p[vidx]
        elif dir==2 or dir == 3:
            v = float('-inf')
            if dir == 2: #正后方
                mind = -CAR_W/2 - self.baseLinkTrans[0][1] + CHK_W#y 轴最小  
                maxd = CAR_W/2  - self.baseLinkTrans[0][1] - CHK_W #y 轴最大
                cidx = 1         #检测坐标y
                vidx = 0         #检测值 x
                vmax = -CAR_H/2 - self.baseLinkTrans[0][0] #最小值过滤
            elif dir == 3: #右侧
                mind = -CAR_H/2 - self.baseLinkTrans[0][0] + CHK_H
                maxd = CAR_H/2 - self.baseLinkTrans[0][0] - CHK_H
                cidx = 0         #检测坐标x
                vidx = 1         #检测值 y
                vmax = -CAR_W/2 - self.baseLinkTrans[0][1] #最小值过滤
            for p in pc:
                if p[cidx] >= mind and p[cidx] <= maxd:
                    if p[vidx] > vmax + 0.05:
                        continue
                    if p[vidx] > v:
                        v = p[vidx]
        return v
 
    def getObstacleDistance(self, dir=0):
        dis = self.clacMiniDistance(dir)
        sub = (CAR_H/2, CAR_W/2, CAR_H/2,CAR_W/2)
        dis = abs(dis)  -  sub[dir]
        if dis < 0:
            dis = 0
        return dis
 
    def __getDistance(self, msg, ang):
        while ang >= math.pi*2:
            ang -= math.pi*2
        while ang < 0:
            ang += math.pi*2
        ii = int(ang / msg.angle_increment+0.5)
        if ii == 0:
            a = [msg.ranges[len(msg.ranges)-1], msg.ranges[0], msg.ranges[1]]
        elif ii == len(msg.ranges)-1:
            a = [msg.ranges[len(msg.ranges)-2], msg.ranges[len(msg.ranges)-1], msg.ranges[0]]
        else:
            a = [x for x in msg.ranges[ii-1:ii+2]]#.sort()
        a.sort()
        return a[2]

    def __scanCallback(self, msg):
        #cloud_out = self.laserProj.projectLaser(msg)
        #print cloud_out
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
        self.__pointcloud = self.laserProj.projectLaser(msg)
        #minDis = (float('inf'),0)
        #for aa in range(-45, 45):
        #    angle = aa*math.pi/180-self.baseLinkTrans[1][2]
        #    rr = self.getDistance(msg, angle)
        #    if rr > 0.15 and rr < minDis[0]:
        #        minDis = (rr, aa)
        #self.__miniDis = minDis
   
    def __del__(self):
        self.__scan.unregister()
if __name__ == '__main__':
    import rospy
    rospy.init_node("radar-demo", log_level=rospy.INFO)
    r = PluseRadar()
    
    r.waitRadar(30)
    print r.baseLinkTrans
    while  not rospy.is_shutdown():
        print "%.2f" % r.getObstacleDistance(0)
        print "%.2f" % r.getObstacleDistance(1)
        print "%.2f" % r.getObstacleDistance(2)
        print "%.2f" % r.getObstacleDistance(3)
        time.sleep(0.2)