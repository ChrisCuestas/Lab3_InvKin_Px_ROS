import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *

from scripts.rosCommunication import getkey

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
    # T0 = SE3(0,0,0)
    # step = 1.0
    # # print(trajectories("trax",step,T0,5))
    # # trajectories("tray",step,T0,10)
    # # trajectories("traz",step,T0,10)
    # print(trajectories("rot",step,T0,10))

    T_actual = SE3(0,0,0)
    print(T_actual)
    # Setting the constants for the movement
    translation_step = 1.0
    rotation_step = 0.01
    movement_kind = np.array(['trax','tray','traz','rot'])
    pointer = 0
    n = 10

    # Starting the control loop
    while 1:
        letter = getkey()
        if(letter ==b'w' or letter == b'W'):
            # Change kind of movement forwards with W key
            pointer = (pointer + 1)%4
            print('Changed to {}'.format(movement_kind[pointer]))
        elif(letter ==b's' or letter == b'S'):
            # Change kind of movement backwards with S key
            pointer = (pointer - 1) if pointer>0 else 3
            print('Changed to {}'.format(movement_kind[pointer]))
        elif(letter ==b'd' or letter == b'D'):
            # Do the selected movement in the positive direction with D key
            print('{} +{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            path = trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,T_actual,n)

            # Calculating the inv kinematics of each pose and sending the result vector to the robot
            # i = 0
            # while i<n:
            #     qs = q_invs(l, path[i])[1]
            #     move(qs)
            #     i = i + 1
            # # Set the new and pose as the actual pose
            # T_actual = path[n]

        elif(letter ==b'a' or letter == b'A'):
            # Do the selected movement in the negative direction with A key
            print('{} -{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            path = trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,T_actual,n)

            # Calculating the inv kinematics of each pose and sending the result vector to the robot
            # i = 0
            # while i<n:
            #     qs = q_invs(l, path[i])[1]
            #     move(qs)

            # # Set the new and pose as the actual pose
            # T_actual = path[n]

        if (letter==b'\x1b'):
            # Escape with crtl+z
            break