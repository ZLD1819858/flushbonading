#!/usr/bin/python
# -*- coding: utf-8 -*-


from copy import deepcopy
import math
import threading
import numpy as np
import sys
import time

from threading import Lock, Event
 
#from trafficsign_rk3399 import detection as detection
from trafficsign_rk1808 import detection as detection

import cv2

this = sys.modules[__name__]

this.__clientCall = []
def init(useRosCam=False):
    this.bridge = None

    if useRosCam:
        import rospy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge, CvBridgeError
        #import tf
        this.bridge = CvBridge()
        this.subImg = rospy.Subscriber("/camera/color/image_raw", Image, __imgCallback)
        # self.subDep = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depCallback)
    else:
        t = threading.Thread(target=__pollcam)
        t.setDaemon(True)
        t.start()

def register(call):
    this.__clientCall.append(call)
def unregister(call):
    this.__clientCall.remove(call)


def __pollcam():
    cap = cv2.VideoCapture(0)
    #cap.set(3, 320)
    #cap.set(4, 240)
    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(3)
            continue
        __imgCallback(frame)
        time.sleep(0.05)


def __imgCallback(data):
        try:
            if this.bridge != None:
                frame = this.bridge.imgmsg_to_cv2(data, "bgr8")
            else:
                frame = data
        except CvBridgeError as e:
            print(e)
            return
        bt = time.time()
        ret = detection.trafficdetect(frame) #frame, rets, types, pp
        et = time.time()
        #print 'time', et-bt
        if ret != None:
            frame, rets, types, pp = ret
            for call in this.__clientCall:
                call(frame, rets, types, pp)
        cv2.imshow("traffic sign", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    init(False)
    def onTrafficssign(frame, rets, types, pp):
        for i in range(len(types)):
            print types[i], rets[i]
    register(onTrafficssign)
    time.sleep(600)
