import numpy as np
from scipy.spatial.transform import Rotation as R

# v1 v2 are 3 dimensitiona points, 
# the function returns Length, Heading (Yaw), Pitch, and Roll angles
# along X-direction, 
# namely, obtain Hpr parameter that converts 1,0,0 -> v
def getLHPRFromVector(v):
    L = np.linalg.norm(np.array(v))

    if L == 0:
        return np.nan

    if v[1] == 0 and v[2] == 0:
        return L, 0.0, 0.0, 0.0
    
    e = np.array([1, 0, 0])
    L = np.linalg.norm(np.array(v))
    x = np.array(np.array(v)/L)
    ang = np.arccos(np.dot(e,x))
    v = np.cross(e,x)
    v = v / np.linalg.norm(v)
    rotation = R.from_rotvec(ang*v)
    yaw,pitch,roll = rotation.as_euler("ZXY", degrees=True)

    return L, yaw, pitch, roll

if __name__ == '__main__':
    #R = getLHPRFromPoints([0,0,0],[1,1,1])
    #print(R)

    #r = R.from_euler("ZYX", (45, 0, 0), degrees=True)
    #print(r.as_matrix())

    #X = np.dot(r.as_matrix(),np.array([1,0,0]))
    #print(X)

    L,H,p,r = getLHPRFromPoints([0,0,0],[0,1,0])

    print(L, H, p, r)