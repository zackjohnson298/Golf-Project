import numpy as np

def Quat2Mat(q):
    R11 = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2
    R12 = 2*q[1]*q[2] - 2*q[0]*q[3]
    R13 = 2*q[1]*q[3] + 2*q[0]*q[2]
    R21 = 2*q[1]*q[2] + 2*q[0]*q[3]
    R22 = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2
    R23 = 2*q[2]*q[3] - 2*q[0]*q[1]
    R31 = 2*q[1]*q[3] - 2*q[0]*q[2]
    R32 = 2*q[2]*q[3] + 2*q[0]*q[1]
    R33 = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
    return np.matrix([[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]])

def Qproduct(a,b,outType = 'list'):
    c = [0,0,0,0]
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    if outType == 'list':
        return c
    else:
        return np.array(c)

def Qconj(q):
    return [q[0],-q[1],-q[2],-q[3]]

def Qrotate(q,v):
    if len(v) == 3:
        vNew = [0,v[0],v[1],v[2]]
    else:
        vNew = v
    vRot = Qproduct(Qproduct(q,vNew),Qconj(q))
    return [vRot[1],vRot[2],vRot[3]]
