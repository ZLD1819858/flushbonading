#!/usr/bin/python
# -*- coding: utf-8 -*-
import copy
import numpy as np
from scipy import spatial

from pyomo.environ import *
from pyomo.dae import *
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt
#%matplotlib inline
# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display
import time
plt.ion()

 

def getcwps(nwps, wt, rp):
    _, nindex = wt.query(rp)
    cwps = np.zeros((5,2))
    if nindex +5 >= len(nwps):
        return False, cwps
    
    for i in range(5):
        cwps[i] = nwps[(nindex+i)%len(nwps)]
        
#     if (nindex + 5) >= 100:
#         cwps[0:100-nindex-1] = nwps[nindex:-1]
#         cwps[100-nindex-1:-1] = nwps[0:nindex+5-100]        
#     else:
#         cwps = nwps[nindex:nindex+5]
    return True, cwps    

def cubic_fun(coeffs, x):
    return coeffs[0]*x**3+coeffs[1]*x**2+coeffs[2]*x+coeffs[0]    
        
def plot_durations(cwps, prex, prey):
    plt.figure(2)
    plt.clf()
    plt.plot(cwps[:,0],cwps[:,1])
    plt.plot(prex, prey)
    plt.scatter(x, y)
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())
 
        
        
N =  8 #19 # forward predict steps
ns = 5  # state numbers / here: 1: x, 2: y, 3: psi, 4: cte, 5: epsi
na = 2  # actuator numbers /here: 1: steering angle, 2: omega


class MPC(object):
    def __init__(self):
        m = ConcreteModel()
        m.sk = RangeSet(0, N-1)
        m.uk = RangeSet(0, N-2)
        m.uk1 = RangeSet(0, N-3)
        
        m.wg       = Param(RangeSet(0, 3), initialize={0:1., 1:10., 2:100., 3:1000}, mutable=True) 
        m.dt       = Param(initialize=0.1, mutable=True)
        m.ref_v    = Param(initialize=0.1, mutable=True)
        m.ref_cte  = Param(initialize=0.0, mutable=True)
        m.ref_epsi = Param(initialize=0.0, mutable=True)
        m.s0       = Param(RangeSet(0, ns-1), initialize={0:0., 1:0., 2:0., 3:0., 4:0.}, mutable=True)
        m.coeffs   = Param(RangeSet(0, 3), 
                          initialize={0:-0.000458316, 1:0.00734257, 2:0.0538795, 3:0.080728}, mutable=True)
        
        
        m.s      = Var(RangeSet(0, ns-1), m.sk)
        m.f      = Var(m.sk)
        m.psides = Var(m.sk)
        m.uv     = Var(m.uk, bounds=(0, 0.3))
        m.uw     = Var(m.uk, bounds=(-0.78, 0.78))
        
        # 0: x, 1: y, 2: psi, 3: cte, 4: epsi
        m.s0_update      = Constraint(RangeSet(0, ns-1), rule = lambda m, i: m.s[i,0] == m.s0[i])
        m.x_update       = Constraint(m.sk, rule=lambda m, k: 
                                      m.s[0,k+1]==m.s[0,k]+m.uv[k]*cos(m.s[2,k])*m.dt 
                                      if k<N-1 else Constraint.Skip)
        m.y_update       = Constraint(m.sk, rule=lambda m, k: 
                                      m.s[1,k+1]==m.s[1,k]+m.uv[k]*sin(m.s[2,k])*m.dt 
                                      if k<N-1 else Constraint.Skip)
        m.psi_update     = Constraint(m.sk, rule=lambda m, k: 
                                       m.s[2,k+1]==m.s[2,k]+ m.uw[k]*m.dt 
                                       if k<N-1 else Constraint.Skip)     
        m.f_update      = Constraint(m.sk, rule=lambda m, k: 
                                       m.f[k]==m.coeffs[0]*m.s[0,k]**3+m.coeffs[1]*m.s[0,k]**2+
                                       m.coeffs[2]*m.s[0,k]+m.coeffs[3])
        m.psides_update = Constraint(m.sk, rule=lambda m, k: 
                                           m.psides[k]==atan(3*m.coeffs[0]*m.s[0,k]**2
                                                              +2*m.coeffs[1]*m.s[0,k]+m.coeffs[2]))
        m.cte_update     = Constraint(m.sk, rule=lambda m, k: 
                                        m.s[3,k+1]==(m.f[k]-m.s[1,k]+m.uv[k]*sin(m.s[2,k])*m.dt) 
                                       if k<N-1 else Constraint.Skip)

        m.epsi_update    = Constraint(m.sk, rule=lambda m, k: 
                                   m.s[4, k+1]==m.psides[k]-m.s[2,k]+m.uw[k]*m.dt 
                                        if k<N-1 else Constraint.Skip)  
        
        m.cteobj  = m.wg[3]*sum((m.s[3,k]-m.ref_cte)**2 for k in m.sk)
        m.epsiobj = m.wg[3]*sum((m.s[4,k]-m.ref_epsi)**2 for k in m.sk)
        m.vobj    = m.wg[2]*sum((m.uv[k]-0.5)**2 for k in m.uk)
        m.uvobj   = m.wg[1]*sum(m.uv[k]**2 for k in m.uk)
        m.uwobj   = m.wg[1]*sum(m.uw[k]**2 for k in m.uk)
        m.sudobj  = m.wg[0]*sum((m.uv[k+1]-m.uv[k])**2 for k in m.uk1)
        m.suaobj  = m.wg[0]*sum((m.uw[k+1]-m.uw[k])**2 for k in m.uk1) 
        m.obj = Objective(expr = m.cteobj+m.epsiobj+m.vobj+m.uvobj+m.uwobj+m.sudobj+m.suaobj, sense=minimize)
        
        self.iN = m#.create_instance()
        
    def Solve(self, state, coeffs):        
        self.iN.s0.reconstruct({0:state[0], 1: state[1], 2:state[2], 3:state[3], 4:state[4]})
        self.iN.coeffs.reconstruct({0:coeffs[0], 1:coeffs[1], 2:coeffs[2], 3:coeffs[3]})
        self.iN.f_update.reconstruct()
        self.iN.s0_update.reconstruct()
        self.iN.psides_update.reconstruct()
        SolverFactory('ipopt').solve(self.iN)
        x_pred_vals = [self.iN.s[0,k]() for k in self.iN.sk]
        y_pred_vals = [self.iN.s[1,k]() for k in self.iN.sk]
        pre_path = np.zeros((N,2))
        pre_path[:,0] = np.array(x_pred_vals)
        pre_path[:,1] = np.array(y_pred_vals)        
        v = self.iN.uv[0]()
        w = self.iN.uw[0]()                                     
        return pre_path, v, w      
        

        
def goWithPath(paths, car):
    x = []
    y = []
    
    nwps = []
    for path in paths:
        for p in path.path:
            x.append(p.x)
            y.append(p.y)
            nwps.append([p.x, p.y])
        
    #t = np.linspace(0, 1, num=len(x))
    #f1 = interp1d(t,x,kind='cubic')
    #f2 = interp1d(t,y,kind='cubic')
    #newt = np.linspace(0,1,100)
    #nwps = np.zeros((100, 2))
    #nwps[:,0] = f1(newt)
    #nwps[:,1] = f2(newt)
 
    wpstree = spatial.KDTree(nwps)

    rp = np.zeros(3) 
    crv = 0.0
    crw = 0.0
    mpc = MPC() 
   
    while True:
        rp[0], rp[1], rp[2] = car.currentPos()
        ret, cwps = getcwps(nwps, wpstree, rp[0:2])
        if not ret:
            break
        px = rp[0] + crv*np.cos(rp[2])*0.1
        py = rp[1] + crw*np.sin(rp[2])*0.1
        psi = rp[2] + crw*0.1
        
        rp[0] = px
        rp[1] = py
        rp[2] = psi
        
        cwps_robot = np.zeros((len(cwps), 2))  #实际路径
        
        for i in range(len(cwps)):
            dx = cwps[i,0] - px
            dy = cwps[i,1] - py
            
            cwps_robot[i,0] = dx*np.cos(psi) + dy*np.sin(psi)
            cwps_robot[i,1] = dy*np.cos(psi) - dx*np.sin(psi)
            
        coeffs = np.polyfit(cwps_robot[:,0], cwps_robot[:,1], 3)
        cte = cubic_fun(coeffs, 0)
        
        f_prime_x = coeffs[2]
        epsi = np.arctan(f_prime_x)
        s0 = np.array([0.0, 0.0, 0.0, cte, epsi])
        pre_path, v, w = mpc.Solve(s0, coeffs) #预测路径
         
        print(v,w)
        car.updateSpeed(v, w);
        crv = v
        crw = w
        time.sleep(0.1)        
            
    
 