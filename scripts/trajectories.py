import numpy as np
import roboticstoolbox as rtb

def trajectories_lin(move_kind, step, T0, n):
    
    if move_kind == "trax":
        np.array(
            [0, 0, 0, step],
            [0, 0, 0,    0],
            [0, 0, 0,    0],
            [0, 0, 0,    1]
        )
    elif move_kind == "tray":

    T1 = T0 + 
    rtb.ctraj(T0,T1,n)