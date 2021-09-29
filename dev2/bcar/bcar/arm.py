#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import sys
from geometry_msgs.msg import PoseStamped, Pose
import time
import tf
from std_msgs.msg import Int32
from copy import deepcopy
import math
this = sys.modules[__name__]

def goPose(a, wait=True, tmout = 8):
    '''
       设置机械臂目的位置, 
       a: 长度为6的数组
           a[0],a[1],a[2], 分别为目的x，y,z 坐标
           a[3],a[4],a[5]， 为机械爪的姿态绕x,y，z轴的旋转
    '''
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # group.get_pose_reference_frame()
    #target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = a[0] #x
    target_pose.pose.position.y = a[1] #y
    target_pose.pose.position.z = a[2] #z
    # 自带吸盘的四元数
    q = tf.transformations.quaternion_from_euler(a[3], a[4], a[5]) #RPY

    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]

    arm.set_pose_target(target_pose, arm.get_end_effector_link())
    return arm.go(True)

def getPose():
    return arm.get_current_pose(arm.get_end_effector_link())
def getRpy():
    return arm.get_current_rpy(arm.get_end_effector_link())
    
def init():
    this.arm = MoveGroupCommander("manipulator")
    this.gripper = rospy.Publisher('/xcar/gripper', Int32, queue_size=0)

def setGripper(en):
    '''
        机械爪控制en:True 抓取
                    :False 释放
    '''
    if not en:
        this.gripper.publish(Int32(data=70))
    else:
        this.gripper.publish(Int32(data=-20))



def goHome(wait=True):
    '''
        机械臂回到初始位置
    '''
    arm.set_named_target('home')
    arm.go(wait)

if __name__ == '__main__':
    print 'arm'
    rospy.init_node("arm_debug", log_level=rospy.INFO)
    init()
    time.sleep(1)
    setGripper(False)
    time.sleep(1)
    print getPose()
    print getRpy()
    goPose([0.30,0,0.2, -3.14,0.7,-3.14])
    time.sleep(1)
    print getPose()
    print getRpy()
    setGripper(True)
    time.sleep(1)
    goHome()
    time.sleep(1)

