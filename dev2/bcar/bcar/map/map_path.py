#!/usr/bin/env python
# -*- coding:  UTF-8 -*-
import random
import math
import copy

class Point:
    def __init__(self, x, y, tag="", yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.__tag = tag
        
    def equal(self, p):
        return abs(self.x - p.x)<0.001 and abs(self.y - p.y)<0.001
    
    def tag(self):
        return self.__tag
    def __str__(self):
        return "(%.2f,%.2f,%.2f,%s)"%(self.x,self.y,self.yaw,self.__tag)
        
class Path:
    def __init__(self, path, name="", trafficlight=False, voice=None, leftp=[], straightp=[], rightp=[]):
        '''
        路径定义
        path： 路径点数组， 
        name:  道路名字
        trafficlight: 是否有交通灯需要识别
        voice：语音播报
        '''
        if name == "":
            name = path[0].tag()+"->"+path[-1].tag()
            
        self.path = path
        self.name = name
        self.trafficlight = trafficlight
        self.voice = voice
        
        self.leftp=leftp
        self.straightp = straightp
        self.rightp = rightp
        
    def tag(self):
        t = []
        for p in self.path:
            t.append(p.tag())
        return "-->".join(t)
        
def __rotaPoint(p, angle, ro="", rn=""):
    '''将x，y逆时针旋转angle后的新坐标'''
    nx = p.x*math.cos(angle) - p.y * math.sin(angle)
    ny = p.x*math.sin(angle) + p.y * math.cos(angle)
    nyaw = p.yaw + angle
    nn = p.tag().replace(ro, rn)
    return Point(nx, ny, nn, nyaw)
    
def __rotaPath(ps, ang, ro="", rn=""):
    '''通过旋转创建镜像道路
        ang: 旋转度数
    '''
    nps = []
    for p in ps:
        np = __rotaPoint(p, ang, ro, rn)
        nps.append(np)
    return tuple(nps)
    
def __rotaPaths(pss, ang, ro, rn):
    npss = []
    for ps in pss:
        p = Path(__rotaPath(ps.path, ang, ro,rn), ps.name.replace(ro,rn), ps.trafficlight, ps.voice)
        npss.append(p)
    return tuple(npss)
'''
    地图中心为（0，0）点方向正北为x轴
    地图长6.4m，宽 6.4m
    宽与为y轴平行，高与x轴平行
'''
MAP_ROAD_WIDTH = 0.7

'''路径点定义'''
Start = Point(0,0,"Start", 0)
A1 = Point(0.70, -0.1, "A1")
A2 = Point(A1.x, A1.y-0.525, "A2")
A3 = Point(A1.x, A1.y-0.525*2, "A3")
A4 = Point(A1.x, A1.y-0.525*3, "A4")
A5 = Point(A1.x, A1.y-0.525*4, "A5")

B1 = Point(A1.x+0.525, A1.y, "B1")
B3 = Point(A1.x+0.525, A1.y-0.525*2, "B3")
B5 = Point(A1.x+0.525, A1.y-0.525*4, "B5")

C1 = Point(A1.x+0.525*2, A1.y, "C1")
C2 = Point(A1.x+0.525*2, A1.y-0.525, "C2")
C3 = Point(A1.x+0.525*2, A1.y-0.525*2, "C3")
C4 = Point(A1.x+0.525*2, A1.y-0.525*3, "C4")
C5 = Point(A1.x+0.525*2, A1.y-0.525*4, "C5")

D1 = Point(A1.x+0.525*3, A1.y, "D1")
D3 = Point(A1.x+0.525*3, A1.y-0.525*2, "D3")
D5 = Point(A1.x+0.525*3, A1.y-0.525*4, "D5")

E1 = Point(A1.x+0.525*4, A1.y, "D1")
E2 = Point(A1.x+0.525*4, A1.y-0.525, "D2")
E3 = Point(A1.x+0.525*4, A1.y-0.525*2, "D3")
E4 = Point(A1.x+0.525*4, A1.y-0.525*3, "D4")
E5 = Point(A1.x+0.525*4, A1.y-0.525*4, "D5")

 
'''增加抓取放置点'''
TEMP_D = 0.05
FA1 = Point(A2.x+TEMP_D, A2.y, "FA1", 0)
FA2 = Point(B3.x, B3.y+TEMP_D, "FA2", math.pi/2)
FA3 = Point(C2.x-TEMP_D, C2.y, "FA3", math.pi)
FA4 = Point(B1.x, B1.y-TEMP_D, "FA4", -math.pi/2)

FB1 = Point(A4.x+TEMP_D, A4.y, "FB1", 0)
FB2 = Point(B5.x, B5.y+TEMP_D, "FB2", math.pi/2)
FB3 = Point(C4.x-TEMP_D, C4.y, "FB3", math.pi)
FB4 = Point(B3.x, B3.y-TEMP_D, "FB4", -math.pi/2)

FC1 = Point(C2.x+TEMP_D, C2.y, "FC1", 0)
FC2 = Point(D3.x, D3.y+TEMP_D, "FC2", math.pi/2)
FC3 = Point(E2.x-TEMP_D, E2.y, "FC3", math.pi)
FC4 = Point(D1.x, D1.y-TEMP_D, "FC4", -math.pi/2)
 
FD1 = Point(C4.x+TEMP_D, C4.y, "FD1", 0)
FD2 = Point(D5.x, D5.y+TEMP_D, "FD2", math.pi/2)
FD3 = Point(E4.x-TEMP_D, E4.y, "FD3", math.pi)
FD4 = Point(D3.x, D3.y-TEMP_D, "FD4", -math.pi/2)

#取货点
F00 = Point(A5.x, A5.y+0.2, "F00", -math.pi/2)

FPoints = [F00, FA1,FA2,FA3,FA4]

FPoints += [FB1,FB2,FB3,FB4]
FPoints += [FC1,FC2,FC3,FC4]
FPoints += [FD1,FD2,FD3,FD4]

FPoints = tuple(FPoints)

Parks = [Start]
Parks = tuple(Parks)



EndPoins = tuple(FPoints+Parks) #路径终点
'''路径定义'''
MAP_PATHS = (
    Path((Start, A1)),
    
    Path((A1, A2)),
    Path((A1, B1)),
    Path((A1, Start)),
    
    Path((A2, A3)),
    Path((A2, A1)),
    
    Path((A3, A2)),
    Path((A3, B3)),
    Path((A3, A4)),
    
    Path((A4, A3)),
    Path((A4, A5)),
    
    Path((A5, A4)),
    Path((A5, B5)),
    
    Path((B1, A1)),
    Path((B1, C1)),
    
    Path((B3, A3)),
    Path((B3, C3)),
    
    Path((B5, A5)),
    Path((B5, C5)),
    
    Path((C1, B1)),
    Path((C1, C2)),
    Path((C1, D1)),
    
    Path((C2, C1)),
    Path((C2, C3)),
    
    Path((C3, C2)),
    Path((C3, D3)),
    Path((C3, C4)),
    Path((C3, B3)),
    
    Path((C4, C3)),
    Path((C4, C5)),
    
    Path((C5, C4)),
    Path((C5, D5)),
    Path((C5, B5)),
    
    Path((D1, C1)),
    Path((D1, E1)),
    
    Path((D3, C3)),
    Path((D3, E3)),
    
    Path((D5, C5)),
    Path((D5, E5)),
    
    Path((E1, D1)),
    Path((E1, E2)),
    
    Path((E2, E1)),
    Path((E2, E3)),
    
    Path((E3, E2)),
    Path((E3, D3)),
    Path((E3, E4)),
    
    Path((E4, E3)),
    Path((E4, E5)),
    
    Path((E5, E4)),
    Path((E5, D5)),
    
    #抓取点路径
    Path((A2, FA1)),
    Path((FA1, A2)),
    Path((B3, FA2)),
    Path((FA2, B3)),
    Path((C2, FA3)),
    Path((FA3, C2)),
    Path((B1, FA4)),
    Path((FA4, B1)),
    
    Path((A4, FB1)),
    Path((FB1, A4)),
    Path((B5, FB2)),
    Path((FB2, B5)),
    Path((C4, FB3)),
    Path((FB3, C4)),
    Path((B3, FB4)),
    Path((FB4, B3)),
    
    Path((C2, FC1)),
    Path((FC1, C2)),
    Path((D3, FC2)),
    Path((FC2, D3)),
    Path((E2, FC3)),
    Path((FC3, E2)),
    Path((D1, FC4)),
    Path((FC4, D1)),
    
    Path((C4, FD1)),
    Path((FD1, C4)),
    Path((D5, FD2)),
    Path((FD2, D5)),
    Path((E4, FD3)),
    Path((FD3, E4)),
    Path((D3, FD4)),
    Path((FD4, D3)),
    
    #取货点
    Path((A4, F00)),
    Path((F00, A5)),
)



def __getFirstPathByDepth(start, end):
    '''深度优先返回第一条找到的路径'''
    pathused = [ 0 for x in range(len(MAP_PATHS))]    #保存以访问的路径
    
    def _getpath(s, e, used):
        for i in range(len(MAP_PATHS)):
            if used[i] == 0 and MAP_PATHS[i].path[0].equal(s):
                used[i] = 1
                if MAP_PATHS[i].path[-1].equal(e):
                    return [i]
                else:
                    for park in EndPoins: #如果为停车位，返回路径为空
                        if park.equal(MAP_PATHS[i].path[-1]):
                            return []
                    ret = _getpath(MAP_PATHS[i].path[-1], e, used)
                    if len(ret) != 0:
                        
                        ret.append(i)
                        return ret
        return []
        
    return _getpath(start, end, pathused)[::-1]
    

def __getFirstPathBySpan(start, end,checkPark = True):
    '''广度优先返回第一条找到的路径,此处广度优先搜索到的第一条路径'''
    pathused = [ 0 for x in range(len(MAP_PATHS))]    #保存以访问的路径
    wkque = []

    def _getnext(pa, s, used):
        '''获取当前节点子节点
            pa:从根节点到当前节点路径
            s：当前节点
        '''
        class node:
            def __init__(self, no, path):
                self.no = no
                self.path = path
                
        sps = []
        for i in range(len(MAP_PATHS)):
            if used[i] == 0 and MAP_PATHS[i].path[0].equal(s):
                path = copy.deepcopy(pa)  #复制节点路径
                path.append(i)            #生成字节的路径
                sps.append(node(i, path))
        return sps
    wkque = wkque + _getnext([], start, pathused)
    while len(wkque)>0:
        i = wkque[0]
        del wkque[0]
        if MAP_PATHS[i.no].path[-1].equal(end):  
            return i.path
        else:
            isPark = False
            if checkPark:
                for park in EndPoins: #如果为停车位，返回路径为空
                    if park.equal(MAP_PATHS[i.no].path[-1]):
                        isPark = True
            if not isPark:
                nd = _getnext(i.path, MAP_PATHS[i.no].path[-1], pathused)
                wkque = wkque + nd
    return []

    

def getFirstPathByDepth(start, end):
    path = []
    pi = __getFirstPathByDepth(start, end)
    for i in pi:
        path.append(MAP_PATHS[i])
    return path
def getFirstPathBySpan(start, end,checkPark=True):
    path = []
    pi = __getFirstPathBySpan(start, end, checkPark)
    for i in pi:
        path.append(MAP_PATHS[i])
    return path




def linePoint(p1, p2, per):
    #计算线段上的点坐标
    x = p1.x + (p2.x - p1.x) * per
    y = p1.y + (p2.y - p1.y) * per
    return Point(x,y)
def distence(a, b):
    #计算两点距离
    dx = a.x-b.x
    dy = a.y - b.y
    return math.sqrt(dx*dx + dy*dy)
    
def cuve(p1, p2, p3, number=10):
    '''
    贝塞尔曲线生成
    p1 起点
    p2 中间点
    p3 终点
    number: 插入点个数-1 默认10 实际插入点9个
    '''
    ret = []
    for t in range(number):
        per = float(t)/number
        sp1 = linePoint(p1, p2, per)
        ep1 = linePoint(p2, p3, per)
        cp  = linePoint(sp1, ep1, per)
        ret.append(cp)
    ret.append(p3)
    return ret
    
def path2cuvexxxx(path, DISTENCE=0.3):
    '''折线路径曲线化, 将多条路径曲线化为一条路径'''
    def pathPoint(path):
        '''通过迭代返回path中的每一个路径点'''
        i = -1
        for p in path:
            i += 1
            for x in p.path[:-1]:
                yield  i, x
            
        yield  i, path[-1].path[-1]
 
    if len(path) == 1 and len(path[0].path) <= 2:
        #只有起始点的路径不需要做曲线化
        return path 
    
    rpath = []
    npoint = []
    pointNUM = 0
    lidx = 0
    flag = 0
    for idx, p3 in pathPoint(path):
        if pointNUM == 0:
            p1 = p3 #pathPoint(path)
            npoint.append(p1) #添加起点
         
            pointNUM += 1
        elif pointNUM == 1:
            p2 = p3 #pathPoint(path)
            pointNUM += 1
     
        else:
             
            cp1 = linePoint(p1, p2, 0.5) #计算p1，p2 中点坐标
            cp3 = linePoint(p2, p3, 0.5) #计算p2，p3 中点坐标
            d12 = distence(cp1,p2)      
            if d12 > DISTENCE:      
                #如果cp1离p2的距离大于DISTENCE，将cp1设置为离p2 DISTENCE处
                cp1 = linePoint(cp1, p2, (d12-DISTENCE)/d12)
            d23 = distence(p2,cp3)
            if d23 > DISTENCE:
                #如果cp3离p2的距离大于DISTENCE，将cp3设置为离p2 DISTENCE处
                cp3 = linePoint(p2, cp3, DISTENCE/d23)
            cv = cuve(cp1, p2, cp3)
            if flag == 1: 
                npoint += cv[:len(cv)/2]
                rpath.append(Path(npoint,path[idx].name, path[idx].trafficlight, path[idx].voice))
                npoint = cv[len(cv)/2:]
                flag = 0
            else:
                npoint += cv
            if lidx != idx:
                flag = 1
            lidx = idx
            p1 = p2
            p2 = p3
            
    npoint.append(p3) #添加终点
    rpath.append(Path(npoint,path[idx].name, path[idx].trafficlight, path[idx].voice))
    
    return rpath


def path2cuve(path, DISTENCE=0.3):
    '''将一条路径转换为曲线'''
    if len(path.path) <= 2:
        return path
    npoint = []
    p1 = path.path[0]
    p2 = path.path[1]
    
    
    npoint.append(p1)
    for p3 in path.path[2:]:
        cp1 = linePoint(p1, p2, 0.5) #计算p1，p2 中点坐标
        cp3 = linePoint(p2, p3, 0.5) #计算p2，p3 中点坐标

        d12 = distence(cp1,p2)      
        if d12 > DISTENCE:      
            #如果cp1离p2的距离大于DISTENCE，将cp1设置为离p2 DISTENCE处
            print d12,'chang cp1', cp1.x,
            cp1 = linePoint(p1, p2, (d12*2-DISTENCE)/(d12*2))
            print cp1.x
      
        
        d23 = distence(p2,cp3)
        if d23 > DISTENCE:
            #如果cp3离p2的距离大于DISTENCE，将cp3设置为离p2 DISTENCE处
            print d23, 'chang cp3', cp3.x,
            cp3 = linePoint(p2, p3, (DISTENCE)/(d23*2))
            print cp3.x
            
        npoint += cuve(cp1, p2, cp3)
   
            
        p1 = p2
        p2 = p3
    npoint.append(p3)
    return Path(npoint) 
    
def path2cuveEx(path, DISTENCE=0.3):
    '''将路径曲线化，路径条数与原路径对应'''
    #ca = []
    cpath = copy.deepcopy(path)
    cpath[0].path = list(path2cuve(path[0]).path)
    if len(path) > 1:
        lp = path[0]
        idx = 0
        del cpath[0].path[-1]  #删除最后一个点
        for pp in path[1:]:
            #print pp.tag()
            #print pp.path[0].x, pp.path[0].y, pp.path[-1].x, pp.path[-1].y
            if distence(lp.path[-1], pp.path[0]) > 0.001:
                print 'err', idx, 'distance', distence(lp.path[-1], pp.path[0])
            pt = [lp.path[-2], lp.path[-1], pp.path[1]]
            cv = Path(path2cuve(Path(pt), DISTENCE).path[1:-1])  #连接前后路径曲线化
            #print cv.path[0].x, cv.path[0].y, cv.path[-1].x, cv.path[-1].y
            #ca.append(cv)
            
            cpath[idx].path += cv.path[:len(cv.path)/2]
            cpath[idx+1].path = cv.path[len(cv.path)/2:]
            cpath[idx+1].path += list(path2cuve(pp, DISTENCE).path[1:-1]) #当前路径曲线化
            idx += 1
            lp = pp
        cpath[idx].path.append(lp.path[-1]) #增加终点坐标
 
    return cpath
   
    
def dumpPath(path):
    sp = []
    for p in path:
        if p.trafficlight:
            sp.append( "["+p.tag()+"]" )
        else:
            sp.append( p.tag() )
    print " ".join(sp)
            
if __name__ == '__main__':
    import cv2
    import map
    
    img = map.getImage()
   
    
    dumpPath(getFirstPathByDepth(Parks[3], Parks[5]))
    
    
    dumpPath(getFirstPathBySpan(Parks[3], Parks[5]))
     
    dumpPath(getFirstPathBySpan(Parks[13], Parks[7]))
    paths = getFirstPathBySpan(Parks[0], Parks[8])
    #map.drawPath(img, paths, (0,0,255))
    print 'cuve'
    dumpPath(path2cuveEx(paths, 0.6))
    print paths[6].tag(), paths[7].tag()
    
    s = 6
    e = 7
    map.drawPath(img,  path2cuveEx(paths, 0.3)  , (0,255,0), 4)
    map.drawPath(img,   paths  , (0,0,0), 2)
    cv2.imshow("map", img)
    cv2.waitKey(0)
    cv2.destroyWindow("map")
   
  
















