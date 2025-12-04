#!/usr/bin/env python3
import math, numpy as np
# link lengths [mm]
L1=150.0; L2=120.0; L3=120.0
def deg2rad(d): return d*math.pi/180.0
def rad2deg(r): return r*180.0/math.pi
def fk_numeric(theta):
    j1,j2,j3,j4 = map(deg2rad,theta)
    r = L2*math.cos(j2) + L3*math.cos(j2+j3)
    x = r*math.cos(j1); y = r*math.sin(j1)
    z = L1 + L2*math.sin(j2) + L3*math.sin(j2+j3)
    c,s = math.cos(j1+deg2rad(theta[3])), math.sin(j1+deg2rad(theta[3]))
    return np.array([[ c,-s,0,x],[ s, c,0,y],[0,0,1,z],[0,0,0,1]],dtype=float)
def ik_numeric(x,y,z, elbow="up"):
    j1 = math.atan2(y,x); r = math.hypot(x,y); z0 = z - L1
    D = (r*r + z0*z0 - L2*L2 - L3*L3)/(2*L2*L3)
    if D<-1 or D>1: return None
    j3 = math.atan2(math.sqrt(1-D*D) if elbow=="up" else -math.sqrt(1-D*D), D)
    phi = math.atan2(z0,r)
    psi = math.atan2(L3*math.sin(j3), L2 + L3*math.cos(j3))
    j2 = phi - psi
    return [rad2deg(j1), rad2deg(j2), rad2deg(j3), 0.0]
