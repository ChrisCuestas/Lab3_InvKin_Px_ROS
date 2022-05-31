# General imports
# import termios, sys, os
import numpy as np
from numpy import pi 
# import time

# ROS imports
# import rospy
# from geometry_msgs.msg import Twist
# from turtlesim.srv import TeleportAbsolute, TeleportRelative
# from turtlesim.srv import Spawn

# Peter Corke RTB Toolbox imports
import roboticstoolbox as rtb
# from spatialmath import SE3
from spatialmath.base import *

#Functions imports
from scripts.invKin import q_invs
from scripts.trajectories import trajectories
from scripts.rosCommunication import *

if __name__ == "__main__":
    # jointCommand('', 6, 'Torque_Limit', 600, 0)
    # jointCommand('', 7, 'Torque_Limit', 500, 0)
    # jointCommand('', 8, 'Torque_Limit', 400, 0)
    # # jointCommand('', 9, 'Torque_Limit', 400, 0)
    # q =np.array([0,0,45,60])

    # Setting the robot in home configuration
    q = home()

    # Phantom X model
    l = np.array([14.5, 10.7, 10.7, 9])
    qlims = np.array([[-3*pi/4, 3*pi/4],[-3*pi/4, 3*pi/4],[-3*pi/4, 3*pi/4],[-3*pi/4, 3*pi/4]])
    robot = rtb.DHRobot(
        [rtb.RevoluteDH(alpha=pi/2, d=l[0], qlim=qlims[0,:]),
        rtb.RevoluteDH(a=l[1], offset=pi/2, qlim=qlims[0,:]),
        rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
        rtb.RevoluteDH(qlim=qlims[0,:])],
        name="Px_DH_std")
    robot.tool = transl(l[3],0,0).dot(troty(pi/2).dot(trotz(-pi/2)))
    # print(robot)
    qt = np.deg2rad(q)
    Tt = robot.fkine(qt)
    T_actual=Tt
    
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
            i = 0
            while i<n:
                qs = q_invs(l, path[i])[1]
                move(qs)

            # Set the new and pose as the actual pose
            T_actual = path[n]

        elif(letter ==b'a' or letter == b'A'):
            # Do the selected movement in the negative direction with A key
            print('{} -{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            path = trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,T_actual,n)

            # Calculating the inv kinematics of each pose and sending the result vector to the robot
            i = 0
            while i<n:
                qs = q_invs(l, path[i])[1]
                move(qs)

            # Set the new and pose as the actual pose
            T_actual = path[n]

        if (letter==b'\x1b'):
            # Escape with crtl+z
            break