#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2


mapimg = cv2.imread("map.jpg", cv2.IMREAD_COLOR)
 
cv2.imshow("map", mapimg)
cv2.waitKey(0)
cv2.destroyWindow("map")

if __name__ == '__main__':
    pass
