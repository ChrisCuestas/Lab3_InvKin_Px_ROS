import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *

def trajectories(move_kind, step, T0, n):
    if move_kind == "trax":
        T1 = T0 @ SE3(step,0,0)
    elif move_kind == "tray":
        T1 = T0 @ SE3(0,step,0)
    elif move_kind == "traz":
        T1 = T0 @ SE3(0,0,step)
    else:
        T1 = T0 @ SE3(troty(step,"deg"))   # Rotation over the TCP's O axis
    return rtb.ctraj(T0,T1,n)

if __name__ == "__main__":
    T0 = SE3(0,0,0)
    step = 1.0
    # print(trajectories("trax",step,T0,5))
    # trajectories("tray",step,T0,10)
    # trajectories("traz",step,T0,10)
    print(trajectories("rot",step,T0,10))

    