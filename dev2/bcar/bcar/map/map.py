#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
from copy import deepcopy
import os
import math
dir = os.path.split(os.path.realpath(__file__))[0]
'''
    地图定义
    
                ^  X
                |
                |
         <------ORIGIN
          Y
'''
MAP_IMAGE   = "map.jpg"     #地图图片文件
MAP_WIDTH   = 3.2           #地图宽度
MAP_HEIGHT  = 3.2           #地图高度

image = cv2.imread(dir+"/"+MAP_IMAGE, cv2.IMREAD_COLOR)

IMAGE_WIDTH = image.shape[1]           #图片宽度
IMAGE_HEIGHT =  image.shape[0]         #图片高度

ORIGIN  =   (IMAGE_HEIGHT+24, 70)  #地图原点在图片中的位置


def getImage():
    return deepcopy(image)
    
def mapp2img(p):
    img_x = int(ORIGIN[1] - p.y * IMAGE_WIDTH/MAP_WIDTH)
    img_y = int(ORIGIN[0] - p.x * IMAGE_HEIGHT/MAP_HEIGHT)
    if img_x < 0:
        img_x = 0
    if img_x >= IMAGE_WIDTH:
        img_x = IMAGE_WIDTH-1
    if img_y < 0:
        img_y = 0
    if img_y >= IMAGE_HEIGHT:
        img_y = IMAGE_HEIGHT - 1
    return (img_x, img_y)
    
def drawPath(img, paths, color, width=2):
    start = 0
    for path in paths:
        for p in path.path:
            if start == 0:
                p1 = mapp2img(p)
                start = 1
            else:
                p2 = mapp2img(p)
                co = color
                if path.trafficlight:
                    co = (255-color[0], 255-color[1], 255-color[2])
                cv2.line(img, p1, p2, co, width)
                p1 = p2

def drawPoint(img, pos, radius=3, color =(255,255,0), thickness=-1):
    cp = mapp2img(pos)
    cv2.circle(img, cp, radius, color, thickness)
    
def drawCar(img, x,y,a, color=(255,255,0)):
    class POS:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    lines = [(0.15,0), (-0.1,0.1),(0,0),(-0.1,-0.1),(0.15,0)]
    nlines = []
    for px,py in lines:
        nx = px * math.cos(a) - py * math.sin(a)
        ny = px * math.sin(a) + py * math.cos(a) 
        cp = mapp2img(POS(x+nx, y+ny))
        nlines.append(cp)
    p1 = nlines[0]
    for p2 in nlines[1:]:
        cv2.line(img, p1, p2, color, 2)
        p1 = p2
    
    
    
 



