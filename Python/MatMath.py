import numpy as np

def Skew(v):
    return np.matrix([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
