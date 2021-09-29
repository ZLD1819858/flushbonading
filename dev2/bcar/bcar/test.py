#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import math
import time
from map import map_path
from map import map
import copy
import sys
import threading
from tracking.move_to_pose import move_to_pose
import cv2
import zxbee
import random
this = sys.modules[__name__]
Parks =  map_path.Parks 
FPoints = map_path.FPoints 

this.pluse_radar = None
this.pluse_sona = None
import sound
import platform as pf


class AiCar:
    def __init__(self, car, goWithPath, trafficsign=None):
        self.car = car
        #self.pluse_radar = None
        self.nvPath = []  #原始导航路径
        self.cvPath = []  #曲线化之后的路径
        self.flowPos = None #当前路径前置点 
        
        
        self.targetPos = None
        self.autoMove = False #自动驾驶中
        self.goWithPath = goWithPath
    
        self.randomRun = False      #是否开启随机路径选择
        
        t = threading.Thread(target=self.__ui_map)
        t.setDaemon(True)
        t.start()
        
        t = threading.Thread(target=self.__run)
        t.setDaemon(True)
        t.start()
        
        #self.currentEndPos = Parks[0] #当前路径结束点，下一路径起点
        #car.initPos(Parks[0].x, Parks[0].y, Parks[0].yaw)
        #sound.say(u"设置初始位置为1号停车位")
        
        __t = threading.Thread(target=self.__procSensor)
        __t.setDaemon(True)
        __t.start()
        
        self.tags = []
        self.tags_pp = []
        
        self.next_command = None
        self.trafficsign = trafficsign
        if self.trafficsign != None:
            self.trafficsign.register(self.__trafficsignCallback)
     
    def setInitPosIdx(self, pidx):
        self.car.initPos(Parks[pidx].x, Parks[pidx].y, Parks[pidx].yaw)
        self.currentEndPos = Parks[pidx]
        sound.say(u"设置初始位置为%d号停车位"%(pidx+1))
        
        
    def __trafficsignCallback(self, frame, rets, types, pp):
       
        self.tags = types
        self.tags_pp  = pp
        
    def setCarPos(self, p):
        self.car.initPos(p.x, p.y, p.yaw)
        self.currentEndPos = p
    
    def setTargetPos(self,pos):
        self.targetPos = pos
    
    def waitGo(self):
        while self.targetPos != None:
            time.sleep(1)
            
    #########发送传感器数据###################
    def __onZXBeeRecvCb(self, msg):
        mj = {}
        if msg[0] == '{' and msg[-1] == '}':
            msg = msg[1:-1]
            its = msg.split(",")
            for it in its:
                kv = it.split("=") 
                if kv[0] == 'V0':
                    val = int(kv[1])
                    if val < len(Parks):
                        self.targetPos = Parks[val]
                if kv[0] == 'V1':
                    val = int(kv[1])
                    if val < len(Parks):
                        self.setCarPos(Parks[val])
                        sound.say(u"设置初始位置为%d号停车位"%(val+1))
                if kv[0] == 'A0' and kv[1] == '?':
                    pos = self.car.currentPos()
                    mj["A0"] = "%.1f&%.1f&%.1f"%(pos[0],pos[1],pos[2])
                       
            a=[]
            for k in mj.keys():
                a.append(k+"="+mj[k])
            a.sort()
            msg = "{"+",".join(a)+"}"
            if len(msg) > 2:
                zxbee.sendMessage(msg)

    def __procSensor(self):
        zxbee.setOnRecvMessageCb(self.__onZXBeeRecvCb)
        lpos = (0,0,0)
        lreport = 0
        lpath = None
        while True:
            time.sleep(1)
            pos = self.car.currentPos()
            if abs(pos[0] - lpos[0])>0.1 or abs(pos[1]-lpos[1])>0.1 or abs(pos[2]-lpos[2])>0.3 :
                mj = {
                    "A0":"%.1f&%.1f&%.1f"%(pos[0],pos[1],pos[2])
                }
                a=[]
                for k in mj.keys():
                    a.append(k+"="+mj[k])
                a.sort()
                msg = "{"+",".join(a)+"}"
                zxbee.sendMessage(msg)
            if time.time() - lreport >= 30:
                mj = {
                    "A0":"%.1f&%.1f&%.1f"%(pos[0],pos[1],pos[2])
                }
                map = {
                    "A10":"batvol",
                    "A11":"temp",
                    "A12":"humi",
                    "A13":"fmbP",
                    "A14":"light",
                    "A15":"MP503",
                    "A16":"MP2"
                }
                for k in map:
                    if self.car.sensors.has_key(map[k]):
                        mj[k] = "%.1f"%self.car.sensors[map[k]]
                    
                a=[]
                for k in mj.keys():
                    a.append(k+"="+mj[k])
                a.sort()
                msg = "{"+",".join(a)+"}"
                zxbee.sendMessage(msg)
                lreport = time.time()
                lpos = pos
            
            #上报导航路径
            if False and lpath != self.cvPath:
                lpath = self.cvPath
                pp = []
                for path in lpath:
                    for p in path.path:
                        pp.append("%.2f&%.2f"%(p.x,p.y))
                if len(pp) > 0:
                    dat = ";".join(pp)
                    msg = "{PATH="+dat+"}"
                    print msg
                    zxbee.sendMessage(msg)
                    lreport = time.time()
                    
    #############绘制地图####################
    #### 绘制地图
    def __ui_map(self):
        lnvPath = None
        gpath = []
        
        sp1 = self.car.currentPos()
        while True:
            if lnvPath != self.cvPath:
                lnvPath = self.cvPath
                bgimg = map.getImage()
                map.drawPath(bgimg, self.cvPath, (0,255,0), 2)
                img = copy.deepcopy(bgimg)
                sp1 = self.car.currentPos()
                map.drawCar(img, sp1[0], sp1[1], sp1[2])
                cv2.imshow("map", img)
                cv2.waitKey(1)
                gpath = []
            
            sp2 = self.car.currentPos()
            dx = sp2[0] - sp1[0]
            dy = sp2[1] - sp1[1]
            d = math.sqrt(dx*dx + dy*dy)
            if d > 0.01 or abs(sp2[2]-sp1[2])>0.005:
                img = copy.deepcopy(bgimg)
                if self.flowPos != None:
                    map.drawPoint(img, self.flowPos)
                
                if self.autoMove:
                    gpath.append(map_path.Point(sp2[0], sp2[1]))
                    map.drawPath(img, (map_path.Path(gpath),), (0,0,255), 2)
                    
                    
                    
                map.drawCar(img, sp2[0], sp2[1], sp2[2])
                cv2.imshow("map", img)
                ret = cv2.waitKey(1)
            time.sleep(0.1)


    ##########导航###########################
    def __onPathIn(self, xpath):
        #print xpath.name, xpath.trafficlight, xpath.voice
        self.currentEndPos = xpath.path[-1]
        if  xpath.trafficlight:
            sound.say(u"前方红绿灯。")
        if xpath.voice != None:
            sound.say(xpath.voice)
        self.next_command = None
            
    def __onPathOut(self, xpath):
        while xpath.trafficlight and 'red' in self.tags:
            self.car.updateSpeed(0, 0, 0)
            time.sleep(1)
    
    def __onPathRun(self, xpath):
        if self.next_command == None:
            #['left', 'right', 'stop', 'straight', 'red', 'green']
            if 'left' in self.tags and len(xpath.leftp) > 0:
                self.next_command = 'left' #xpath.leftp[0]
            if 'right' in self.tags and len(xpath.rightp) > 0:
                self.next_command = 'right' #xpath.rightp[0]
            if 'straight' in self.tags and len(xpath.straightp) > 0:
                self.next_command = 'straight' #xpath.straightp[0]  
            #if 'stop' in self.tags:
        x,y,yaw = self.car.currentPos() 
        dx = self.currentEndPos.x - x
        dy = self.currentEndPos.y - y
        d = math.sqrt(dx*dx + dy*dy)
       
        if d < 0.4:
            
            if self.next_command ==None and self.randomRun:
                '''随机获取下一路径点'''
                
                np = xpath.leftp + xpath.straightp + xpath.rightp
                if len(np) != 0:
                    npath = np[random.randint(0,len(np)-1)]
                    np = npath.leftp + npath.straightp + npath.rightp
                    while len(np) == 1:
                        npath = np[0]
                        np = npath.leftp + npath.straightp + npath.rightp
                    self.setTargetPos(npath.path[-1])
            elif self.next_command !=None:
                if self.next_command == 'left':
                    sound.say(u"前方左转。")
                    self.setTargetPos(xpath.leftp[0].path[-1])
                if self.next_command == 'right':
                    sound.say(u"前方右转。")
                    self.setTargetPos(xpath.rightp[0].path[-1])
                if self.next_command == 'straight':
                    sound.say(u"前方直行。")
                    self.setTargetPos(xpath.straightp[0].path[-1])    
                self.next_command = None
                
    def setRandomPath(self, en):
        self.randomRun = en
    
    def __runPath(self, path=None):
        '''path 为未曲线化之前的路径'''
   
        while True:
            print self.currentEndPos, self.targetPos
            path = map_path.getFirstPathBySpan(self.currentEndPos, self.targetPos, True)
            map_path.dumpPath( path )

            self.targetPos = path[-1].path[-1]
            self.nvPath = path
            #self.cvPath = map_path.path2cuveEx(path, 0.3)
            self.cvPath = path
            needSetYaw = not self.autoMove
            self.autoMove = True
           
            
                
            #sound.say(u'准备出发，全程%.1f米,预计用时%.0f分钟。'%(d,d/(0.2*60)))
            endRun = False
            lpath = None  
            
            lastObsTime = None
            for vx, vy, va, xpath, idxpoint in self.goWithPath(self.cvPath, self.car):
                #print "%.2f,%.2f %.2f"%(vx, vy, va),
                self.flowPos = xpath.path[idxpoint]
                 
                if this.pluse_radar != None:
                    def dis2speed(dis, iv):
                        '''行驶中的避障速度'''
                        if dis <= 0.4:
                            iv = 0
                        if dis <= 0.65:
                            if iv >= 0.1:
                                iv = 0.1
                        if dis <= 0.7:
                            if iv >= 0.15:
                                iv = 0.15
                        return iv
                        
                    if vx > 0:
                        dis = this.pluse_radar.getObstacleDistance(0)
                        vx = dis2speed(dis, vx)
                        if abs(vx) < 0.01:
                            va = 0
                            vy = 0
                            vx = 0
                            lastObsTime = time.time()
                    elif vx < 0:
                        dis = this.pluse_radar.getObstacleDistance(2)
                        vx = -dis2speed(dis, -vx)
                    if vy > 0:
                        dis = this.pluse_radar.getObstacleDistance(1)
                        vy = dis2speed(dis, vy)
                    elif vy < 0:
                        dis = this.pluse_radar.getObstacleDistance(3)
                        vy = -dis2speed(dis, -vy)
                

                if this.pluse_sona != None:
                    dis = this.pluse_sona.getDistance()
                    if dis < 0.3:
                        vx = vy = va = 0
                        lastObsDisTime = time.time()
                    if dis < 0.5:
                        if vx >= 0.1:
                            vx = 0.1
                    if dis < 0.7:
                        if vx >= 0.15:
                            vx = 0.15
                            
                if lastObsTime != None: 
                    if time.time()-lastObsTime < 2:
                        vx = 0
                        vy = 0
                        va = 0
                    else:
                        lastObsTime = None
                        
                if 'stop' in self.tags:
                    vx = vy = va = 0
                
                if xpath.trafficlight and 'red' in self.tags:
                    vx = vx * 0.7
                        
                if self.targetPos != path[-1].path[-1]: #目标位置改变
                    #if self.currentEndPos != path[-1].path[-1]: #当前路段终点不是停车场直接退出，否则走完当前路段
                    #    endRun = True
                    break
                if lpath != xpath:
                    if lpath != None:
                        idx = self.cvPath.index(lpath)
                        self.__onPathOut(self.nvPath[idx])
                    idx = self.cvPath.index(xpath)
                    self.__onPathIn(self.nvPath[idx])
                    lpath = xpath
                self.__onPathRun(self.nvPath[idx])
                #print "--->%.2f,%.2f %.2f"%(vx, vy, va)
                self.car.updateSpeed(vx, va, vy)
                time.sleep(0.1)
            self.flowPos = None
            if self.targetPos == path[-1].path[-1]:
                
                move_to_pose(self.car,self.targetPos.x, self.targetPos.y, self.targetPos.yaw, 0.05,0.1)
                self.car.updateSpeed(0,0,0)
                self.car.updateSpeed(0,0,0)
                self.autoMove = False
                self.targetPos = None
                break
            
    
    def __run(self):
         
 
        while True:
            if self.targetPos != None:
                #path = map_path.getFirstPathBySpan(self.currentEndPos, self.targetPos, False)
                #ps = self.car.currentPos()
                #path.insert(0, map_path.Path((map_path.Point(ps[0], ps[1]) ,self.currentEndPos)))
                #self.targetPos = path[-1].path[-1]
                #target = self.targetPos
                self.__runPath()
                
            else:
                time.sleep(1)


def grapObject(pos, size):
    '''移动车辆到目标物体附件待抓取'''
    tf_listener = tf.TransformListener()
    point_trans = tf_listener.waitForTransform("base_link", pos.header.frame_id, rospy.Time(0), rospy.Duration(1))
    pos = tf_listener.transformPoint("base_link", pos)
    xcar_odom.go(pos.point.x - 0.24, pos.point.y, 0, 0.01, 0.02)
    x = 0.25+0.1    #+0.05摄像头超前机械臂夹头7cm,
    y = 0.01
    z = 0.15 +0.06#+ hpos.point.z + size[1]/2 #从顶部抓取 0.15摄像头相对于base_link的高度
    z = z + 0.005  # 0.0  机械臂模型上多了个基座, 如果爪子过高或过低抓不到，通过修改此处微调
    ang = 120
    arm.setGripper(False)
    time.sleep(1)
    print 'arm go', x, y, z
    arm.goPose((x, y, z, 0, ang * math.pi / 180, math.atan(y / x)))
    time.sleep(1)
    arm.setGripper(True)
    time.sleep(1)
    arm.goHome()
    #x = 0.25
    #y = 0.01
    #z = 0.25
    #ang = 135
    #arm.goPose(x,y,z, 0, ang*math.pi/180,math.atan(y/x))

    xcar_odom.go(-(pos.point.x - 0.24),-pos.point.y,0, 0.05, 0.1)
def dropObject():
    '''
        释放药盒
    '''
    pos = PoseStamped() 
    pos.pose.position.x = 0.4
    pos.pose.position.y = 0
    pos.pose.position.z = 0.0
        
    #xcar_odom.go(pos.pose.position.x - 0.3, pos.pose.position.y, 0)
    x = 0.3
    y = 0
    z = 0.15 + pos.pose.position.z   # 物体高度
   
    ang = 140
    arm.goPose((x, y, z, 0, ang * math.pi / 180, math.atan(y / x)))
    time.sleep(1)
    arm.setGripper(False)
    time.sleep(1)
    arm.goHome()
    time.sleep(1)
    arm.setGripper(True)
    time.sleep(1)
    #xcar_odom.go(-(pos.pose.position.x - 0.3), -pos.pose.position.y, 0, 0.05, 0.2)
    
def testAiCar():
    import argparse
    sys.path.append("tracking")
    sys.path.append("car")

    parser = argparse.ArgumentParser(" ".join(sys.argv))
    parser.add_argument('-s','--start', default='0')
    parser.add_argument('-e','--end', default='0')
    #parser.add_argument('-c','--control', default='pure_pursuit_2')
    parser.add_argument('-c','--control', default='simple_move')
    parser.add_argument('-x','--xcar', default='scar')
    parser.add_argument('-t','--trafficsign', default='0')
    
    args = parser.parse_args()
    start = int(args.start)
    end = int(args.end)
    
    sound.say(u"系统启动中。", True)
   
    #usePredict = True           #是否开启交通标志识别
    if args.trafficsign == "1":
        import trafficsign as trafficsign
        trafficsign.init()
    else:
        trafficsign = None
    if args.xcar == "xcar":
        import rospy
        #rospy.init_node("xcar-demo", log_level=rospy.INFO)
        import radar
        #global pluse_radar 
        this.pluse_radar = radar.PluseRadar()
        if not this.pluse_radar.waitRadar():
            sound.say(u"雷达系统异常。", True)
            sound.say(u"请检查雷达系统，并重新启动应用。", True)
            sys.exit(0)
        this.pluse_radar = None
        import sona
       # this.pluse_sona = sona.PluseSona() 
        
        
        
    sound.say(u"一切正常。", True)

    goWithPathEx = __import__(args.control).goWithPathEx
    ACar = __import__(args.xcar).ACar

    car = ACar()
    acar = AiCar(car, goWithPathEx, trafficsign)
    acar.setInitPosIdx(start)
    acar.setTargetPos(FPoints[0])
    acar.waitGo()
    
    print u'目标检测'
    t = testGrap()
    m2p = {
    'apple':1, 'banana':2, 'grape':3, 'eraser':9, 'writing_case':10, 'ink_bottle':11, 'ladder':13, 'table':14, 'chair':15
    }
    pi = random.randint(1, len(FPoints)-1)
    acar.setTargetPos(FPoints[m2p[t]])
   
    print u'go 目标点'
    acar.waitGo()
    move_to_pose(acar.car,FPoints[m2p[t]].x, FPoints[m2p[t]].y, FPoints[m2p[t]].yaw, 0.01,0.05)
    print u'放置物体'
    dropObject()
    
    acar.setTargetPos(Parks[start])
    acar.waitGo()
    print u'任务完成'

def testGrap():
    locobj = locObj.LocObject(detection.pilldetect)
    pos, size, t = locobj.locObject()
    print pos, size, t
    grapObject(pos, size)
    return t
if __name__ == '__main__':
    
    sys.path.append("car")
    import rospy
    import xcar_odom
    import locObj
    import arm
    import tf
    from geometry_msgs.msg import PoseStamped
    
    #from obj_detection_rk3399 import detection
    from fruit_detection import detection 
    rospy.init_node('testGrap', anonymous=True)
    xcar_odom.init()
    arm.init()
    arm.goHome()
    
    for index in range(3):
    	testAiCar()
    	time.sleep(5)
    
    #testGrap()
    #dropObject()

