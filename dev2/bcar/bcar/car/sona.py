#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range

class PluseSona:
    
    def __init__(self):
        
        self.__scan = rospy.Subscriber('/xcar/sonar1', Range, self.__sonacb)
        self.range = 5
        
    def __sonacb(self, msg):
        
        self.range = msg.range
        
    def getDistance(self):
        return self.range
        
if __name__ == '__main__':
    import rospy
    import time
    rospy.init_node("sano-demo", log_level=rospy.INFO)
    p = PluseSona()
    for i in range(60):
        print p.getDistance()
        time.sleep(1)
        