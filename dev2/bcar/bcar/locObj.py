#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tf
from tf.msg import tfMessage
import sys
import time
import os
import math
from threading import Lock
from threading import Lock, Event

this = sys.modules[__name__]
class LocObject:
    '''目标识别，并获取目标三维坐标'''
    def __init__(self, findObjCall, onObjCall=None, MARGIN_PIX=7):

        self.rect = np.array([])
        self.type = ""
        self.onRSObject = onObjCall
        self.findObj = findObjCall
        #self.maxDiffDix = 5
        self.MARGIN_PIX = MARGIN_PIX     #在原检测框里margin

        self.__push = True
        self.__lastTaget = None
        self.__window = False
        self.__target = None

        self.__evt = Event()

        self.bridge = CvBridge()
        self.subImg = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        #self.subDep = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depCallback,queue_size=1)
        '''未与rgb图像对齐的深度数据需要先对齐后再使用'''
        self.subDep = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depCallback,queue_size=1)

    def deep2pos(self, m, n, deep):
        # // 相机内参
        camera_factor = 1000.0;
        camera_cx = 321.798;
        camera_cy = 239.607;
        camera_fx = 615.899;
        camera_fy = 616.468;

        # 计算这个点的空间坐标
        x = deep / camera_factor;
        y = -(n - camera_cx) * x / camera_fx;
        z = -(m - camera_cy) * x / camera_fy;
        return (x, y, z)

    def getPos(self, deep, xx, yy, wp, hp, maxDeep = 1000):
        '''
        '''
        sumx = sumy = sumz = num = 0

        for m in range(int(yy-hp), int(yy+hp)):
            for n in range(int(xx-wp), int(xx+wp)):
                if deep[m][n] < 280:
                    #print("min %d"%deep[m][n])
                    continue
                if deep[m][n] > maxDeep:
                    #print("max %d"%deep[m][n])
                    continue

                (x, y, z) = self.deep2pos(m, n, deep[m][n])
                sumx += x
                sumy += y
                sumz += z
                num += 1
        if num > 0:
            return (sumx/num, sumy/num, sumz/num)
        else:
            return (0,0,0)

    def depCallback(self, data):
        '''
            深度数据回调函数
        '''
        if self.__push :
            return
        depimg = self.bridge.imgmsg_to_cv2(data)

        ret = []
        
        if len(self.rect) > 0 and self.rect[0]-30 > 0:
            '''
                检测到目标，获取目标三维坐标
            '''
            rect = self.rect
            self.rect = []
            rect = (rect[0] - 30, rect[1], rect[2], rect[3]) #简单平移对齐到rgb摄像头
            ret = (x, y, z) = self.getPos(depimg, rect[0] + rect[2] / 2, rect[1] + rect[3] / 2,  self.MARGIN_PIX, self.MARGIN_PIX)
            print rect, "--->", ret
     
            p1 = (x1,y1,z1) = self.getPos(depimg, rect[0], rect[1], self.MARGIN_PIX-1,
                                          self.MARGIN_PIX-1)
            p2 = (x2, y2, z2) = self.getPos(depimg, rect[0]+rect[2], rect[1],  self.MARGIN_PIX - 1,
                                            self.MARGIN_PIX - 1)
            p3 = (x3, y3, z3) = self.getPos(depimg, rect[0], rect[1]+rect[3],  self.MARGIN_PIX - 1,
                                            self.MARGIN_PIX - 1)
            p4 = (x4, y4, z4) = self.getPos(depimg, rect[0]+rect[2], rect[1]+rect[3],  self.MARGIN_PIX - 1,
                                            self.MARGIN_PIX - 1)

            w = ((y1 - y2) + (y3 - y4)) / 2 * ((rect[2]+self.MARGIN_PIX*2.0)/rect[2])
            h = ((z1 - z3) + (z2 - z4)) / 2 * ((rect[3]+self.MARGIN_PIX*2.0)/rect[3])
            size = (w, h)
            
            
            rect2 = (rect[0]+self.MARGIN_PIX, rect[1]+self.MARGIN_PIX, rect[2]-self.MARGIN_PIX*2, rect[3]-self.MARGIN_PIX*2)
            #cv2.rectangle(depimg, (rect2[0], rect2[1]),
            #              (rect2[0] + rect2[2], rect2[1] + rect2[3]), (255, 255, 255), 2)
            #cv2.rectangle(depimg, (rect[0],rect[1]), (rect[0] + rect[2], rect[1] + rect[3]),(255, 255, 255), 2)
                
        if len(ret)>0 and (ret[0] + ret[1] + ret[2]) != 0:
            poi = PointStamped()
            '''计算出来的坐标实际是相对于camera rgb的，此处要转换成camera_link 直接在y上减去一个0.03'''
            poi.header.frame_id = "camera_link" #data.header.frame_id 
            poi.point.x = ret[0]
            poi.point.y = ret[1] - 0.03 #camera rgb坐标转camera link坐标
            poi.point.z = ret[2]
            self.__onRSObject(poi, size)
        #cv2.imshow("depth window", depimg)
        #cv2.waitKey(1)
        
    def callback(self, data):
        '''
            摄像头图片数据回调,获取目标在图片中的位置
        '''
        if self.__push :
            if self.__window:
                cv2.destroyWindow("Image window")
                cv2.waitKey(1)
                self.__window = False
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        _, box, type = self.findObj(cv_image)
        if len(box)>0 :
            rect = box #rect = cv2.boundingRect(box)
            if rect[2]>self.MARGIN_PIX*2 and rect[3] > self.MARGIN_PIX*2:
                rect2 = (rect[0]+self.MARGIN_PIX, rect[1]+self.MARGIN_PIX, rect[2]-self.MARGIN_PIX*2, rect[3]-self.MARGIN_PIX*2)
                self.rect = rect2
                self.type = type
                cv2.rectangle(cv_image, (rect2[0], rect2[1]),
                              (rect2[0] + rect2[2], rect2[1] + rect2[3]), (0, 255, 0), 2)
                cv2.rectangle(cv_image, (rect[0],rect[1]), (rect[0] + rect[2], rect[1] + rect[3]),(0, 0, 255), 2)
            else:
                self.rect = np.array([])
        else:
           self.rect = np.array([])
        self.__window = True
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

    def push(self):
        self.__push = True


    def resume(self):
        self.__target = None
        self.__lastTaget = None
        self.__push = False

    def __onRSObject(self, pos, size):
        '''
            多次计算获取目标的一个稳定坐标
        '''
        #print 'loc object', pos, size
        #tf_listener = tf.TransformListener()
        #point_trans = tf_listener.waitForTransform("base_link", pos.header.frame_id, rospy.Time(0), rospy.Duration(1))
        #pos = tf_listener.transformPoint("base_link", pos)
        if self.__lastTaget == None:
            self.__lastTaget = [(pos, size)]
            return

        if abs(pos.point.x - self.__lastTaget[-1][0].point.x) < 0.006 and \
                abs(pos.point.y - self.__lastTaget[-1][0].point.y) < 0.006 and \
                abs(pos.point.z - self.__lastTaget[-1][0].point.z) < 0.006 and \
                abs(size[0] - self.__lastTaget[-1][1][0]) < 0.01 and \
                abs(size[1] - self.__lastTaget[-1][1][1]) < 0.01 :
            self.__lastTaget.append((pos, size))
            if len(self.__lastTaget) >= 5:
                sumx = sumy = sumz = 0
                size_w = size_h = 0
                for (pos,size) in self.__lastTaget:
                    sumx += pos.point.x
                    sumy += pos.point.y
                    sumz += pos.point.z
                    size_w += size[0]
                    size_h += size[1]

                x = sumx / len(self.__lastTaget)
                y = sumy / len(self.__lastTaget)
                z = sumz / len(self.__lastTaget)
                w = size_w / len(self.__lastTaget)
                h = size_h / len(self.__lastTaget)
                poi = PointStamped()
                poi.header.frame_id = pos.header.frame_id # data.header.frame_id
                poi.point.x = x
                poi.point.y = y
                poi.point.z = z
                if self.onRSObject != None:
                    self.onRSObject(self, poi, (w,h), self.type)
                else:
                    self.__target = (poi, (w,h), self.type)
                    self.__evt.set()
                self.__lastTaget = None
        else:
            self.__lastTaget = [(pos, size)]


    def locObject(self, wait=None):
        '''
            等待目标识别，并获取目标三维坐标
        '''
        self.resume()
        self.__evt.wait(wait)
        self.push()
        obj = self.__target
        self.__target = None
        return obj


    def __del__(self):

        self.subImg.unregister()
        self.subDep.unregister()

if __name__ == '__main__':
    import threading
    #from obj_detection_rk3399 import detection
    from fruit_detection import    detection
    rospy.init_node('obj_detect', anonymous=True)
    
    locobj = LocObject(detection.pilldetect)
    ret, type = locobj.locObject()
    print ret, type
        
    
