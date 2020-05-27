
def Qproduct(a,b):
    c = [0,0,0,0]
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return c

def Qconj(q):
    return [q[0],-q[1],-q[2],-q[3]]

def Qrotate(q,v):
    if len(v) == 3:
        vNew = [0,v[0],v[1],v[2]]
    else:
        vNew = v
    vRot = Qproduct(Qproduct(q,vNew),Qconj(q))
    return [vRot[1],vRot[2],vRot[3]]
