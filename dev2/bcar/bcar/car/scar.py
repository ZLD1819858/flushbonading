#!/usr/bin/python
# -*- coding: utf-8 -*-

 
from copy import deepcopy
import math
import threading
import numpy as np
import sys
import time
 
import os
from random import random
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
class ACar:
    def __init__(self):
        self.__x = 0
        self.__y = 0
        self.__yaw = 0
        
       
        self.speed_x = 0
        self.speed_y = 0
        self.speed_a = 0
      
        self.lock = threading.Lock()
        
        self.__running__ = True
        self.threadLoc = threading.Thread(target=self.__locCar)
        self.threadLoc.setDaemon(True)
        self.threadLoc.start()
        
        self.sensors = {}
        
        ts = threading.Thread(target=self.__sensor)
        ts.setDaemon(True)
        ts.start()
        
    def initPos(self, x, y, yaw):
        self.lock.acquire()
        self.__x = x + (random()-0.5)/2.5
        self.__y = y + (random()-0.5)/2.5
        self.__yaw = yaw + (random()-0.5)/2
        self.lock.release()
      
        
        
    def updateSpeed(self, vx, va=0, vy=0):
    
        self.speed_x = vx
        self.speed_y = vy
        self.speed_a = va
        

    def currentPos(self):
        return (self.__x, self.__y, self.__yaw)
        
    def __sensor(self):
        while True:
            self.sensors['batvol'] =  11.1 +random() #电池电压
            self.sensors['temp'] =    25+random()#环境温度
            self.sensors['humi'] =    50+random()*10#环境湿度
                                            #气压计温度
            self.sensors['fmbP'] =    random()*100  #环境气压
            self.sensors['light'] =   random()*1000#环境光强
            
            self.sensors['MP503'] =  random()*1000;
            self.sensors['MP2'] =    random()*1000;
            self.sensors['Sonar1'] =  random()*1000;
            self.sensors['Sonar2'] =  random()*1000;
            self.sensors['Sonar3'] =  random()*1000;
            time.sleep(10)
            
    def __del__(self):
        self.__running__ = False
        self.threadLoc.join()
        
    def __locCar(self):
        #rate = rospy.Rate(20)
        __vx = 0
        __vy = 0
        __a = 0
        start = time.time()
        while self.__running__:
            if False:
                if self.speed_x - __vx > 0.01:
                    __vx += 0.01
                elif self.speed_x - __vx < 0.01:
                    __vx -= 0.01
                else:
                    __vx = self.speed_x
                
                if self.speed_y > __vy:
                    __vy += 0.01
                elif self.speed_y < __vy:
                    __vy -= 0.01
                else:
                    __vy = self.speed_y
                    
                if self.speed_a - __a > 0.1:
                    __a += 0.08
                elif self.speed_a - __a < 0.1:
                    __a -= 0.08
                else:
                    __a = self.speed_a
            else:
                __vx = self.speed_x
                __vy = self.speed_y
                __a = self.speed_a
                
            self.lock.acquire()
            dt = time.time() - start
            start = time.time()
            if abs(__vx)>0.001 or abs(__vy)>0.001 or abs(__a)>0.01:
                self.__x = self.__x + __vx * np.cos(self.__yaw) * dt #+ (random()-0.5)/50
                self.__y = self.__y + __vx * np.sin(self.__yaw) * dt #+ (random()-0.5)/100
                
                
                self.__x = self.__x + __vy * np.cos(self.__yaw+math.pi/2) * dt #+ (random()-0.5)/50
                self.__y = self.__y + __vy * np.sin(self.__yaw+math.pi/2) * dt #+ (random()-0.5)/100
                
                self.__yaw = normalize_angle(self.__yaw + __a * dt) #+ (random()-0.5)/10
                
            self.lock.release()
            time.sleep(0.1)
             
            