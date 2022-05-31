# General imports
import termios, sys, os
import numpy as np
from numpy import pi 
import time
# ROS imports
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from turtlesim.srv import Spawn
# Peter Corke RTB Toolbox imports
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *
#Functions imports
from scripts.invKin import q_invs
from scripts.trajectories import trajectories
from scripts.rosCommunication import *

# TERMIOS = termios

# def jointCommand(command, id_num, addr_name, value, time):
#     #rospy.init_node('joint_node', anonymous=False)
#     rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
#     try:        
#         dynamixel_command = rospy.ServiceProxy(
#             '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
#         result = dynamixel_command(command,id_num,addr_name,value)
#         rospy.sleep(time)
#         return result.comm_result
#     except rospy.ServiceException as exc:
#         print(str(exc))

# def grad21024 (num):
#     return 512+(num*(511/180))
# def home(q):
#     # q1=np.array([grad21024(q[0]),grad21024(q[1]),grad21024(q[2]),grad21024(q[3])])
#     # jointCommand('', 6, 'Goal_Position', q1[0], 0.5)
#     # jointCommand('', 7, 'Goal_Position', q1[1], 0.5)
#     # jointCommand('', 8, 'Goal_Position', q1[2], 0.5)
#     # jointCommand('', 9, 'Goal_Position', q1[3], 0.5)
#     time.sleep(0.5)
# def move(q):
#     q1=np.array([grad21024(q[0]),grad21024(q[1]),grad21024(q[2]),grad21024(q[3])])
#     jointCommand('', 6, 'Goal_Position', q1[0], 0.5)
#     jointCommand('', 7, 'Goal_Position', q1[1], 0.5)
#     jointCommand('', 8, 'Goal_Position', q1[2], 0.5)
#     jointCommand('', 9, 'Goal_Position', q1[3], 0.5)

# def trajectories(move_kind, step, T0, n):
#     if move_kind == "trax":
#         T1 = T0 @ SE3(step,0,0)
#     elif move_kind == "tray":
#         T1 = T0 @ SE3(0,step,0)
#     elif move_kind == "traz":
#         T1 = T0 @ SE3(0,0,step)
#     else:
#         T1 = T0 @ SE3(troty(step,"deg"))   # Rotation over the TCP's O axis
#     return rtb.ctraj(T0,T1,n)

# def q_invs(l, T):
#     # l=np.array([1, 2, 3, 4])
#     # T=np.matrix([[1, 2, 3, 4],[5, 6, 7, 8],[9, 10, 11, 12],[13, 14, 15, 16]])
#     Pw = T[0:3, 3]-(l[3]*T[0:3, 2])

#     q1a=np.artan2(T[1,3],T[0,3])
#     q1b=np.artan2(-T[1,3],-T[0,3])
#     pxy = np.sqrt(Pw[0]^2 + Pw[1]^2)
#     z = Pw[4] - l[0]
#     r = np.sqrt(pxy^2 + z^2)

#     the3 = np.arccos((r**2 - l[1]**2 - l[2]**2)/(2*l[1]*l[2]))
#     if np.isreal(the3):
#         alp = np.arctan2(z,pxy)
#         the2a = alp - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
#         the2b = alp + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))

#         # Codo abajo
#         q2a = -np.pi/2 + the2a      # q2
#         q3a = the3;                 # q3

#         #Codo arriba
#         q2b = -np.pi/2 + the2b      # q2
#         q3b = -the3;                # q3

#         # Orientacion
#         R_p = ((rotz(q1a)).transpose)@T[0:3,0:3]
#         pitch = np.arctan2(R_p[2,0],R_p[0,0])
#         # Codo abajo
#         q4a = pitch - (q2a + q3a)   # q4
#         # Codo arriba
#         q4b = pitch - (q2b + q3b)   # q4
#     else:
#         q2a = np.NaN
#         q2b = np.NaN
#         q3a = np.NaN
#         q3b = np.NaN
#         q4a = np.NaN
#         q4b = np.NaN

#     q_inv=np.zeros(4)
#     q_inv[0,0:4] = np.array([q1a, q2a, q3a, q4a])
#     q_inv[1,0:4] = np.array([q1a, q2b, q3b, q4b])

#     # Queda invertido el yaw debido a la restriccion de movimiento
#     # Usando q1b
#     q_inv[2,0:4] = np.array([q1b, -q2a, -q3a, -q4a])
#     q_inv[3,0:4] = np.array([q1b, -q2b, -q3b, -q4b])

#     for i in len(q_inv[0,:]):
#         if np.any(np.isnan(q_inv[i,:])):
#             q_inv[i,:] = np.array([np.NaN,np.NaN,np.NaN,np.NaN])
#     return (q_inv)
    
# def getkey():
#     fd = sys.stdin.fileno()
#     old = termios.tcgetattr(fd)
#     new = termios.tcgetattr(fd)
#     new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
#     new[6][TERMIOS.VMIN] = 1
#     new[6][TERMIOS.VTIME] = 0
#     termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
#     c = None
#     try:
#         c = os.read(fd, 1)
#     finally:
#         termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
#     return c

# def toCenter():
#     rospy.wait_for_service('/turtle1/teleport_absolute')
#     try:
#         telA=rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
#         resp=telA(5,5,1.5)

#     except rospy.ServiceException:
#         pass

# def giro180():
#     rospy.wait_for_service('/turtle1/teleport_relative')
#     try:
#         telR=rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
#         rel=telR(0,3.1416)
#     except rospy.ServiceException:
#         pass

# def pubVel(x, z, time):
#     pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     rospy.init_node('velPub', anonymous=True)
#     vel = Twist()
#     vel.linear.x = x
#     vel.angular.z = z
#     rospy.loginfo(vel)
#     fTime = rospy.Time.now() + rospy.Duration(time)
#     while  rospy.Time.now() < fTime:
#         pub.publish(vel)

if __name__ == "__main__":
    # jointCommand('', 6, 'Torque_Limit', 600, 0)
    # jointCommand('', 7, 'Torque_Limit', 500, 0)
    # jointCommand('', 8, 'Torque_Limit', 400, 0)
    # jointCommand('', 9, 'Torque_Limit', 400, 0)
    q =np.array([0,0,45,60])
    home(q)
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
    Tactual=Tt
    


    translation_step = 1.0
    rotation_step = 0.01
    movement_kind = np.array(['trax','tray','traz','rot'])
    pointer = 0
    n = 10

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
            # function(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            Camino=trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,Tactual,n)
            i=0
            while i<n:
                qs=q_invs(l, Camino[i])[1]
                move(qs)
            Tactual=Camino[n]

        elif(letter ==b'a' or letter == b'A'):
            # Do the selected movement in the negative direction with A key
            print('{} -{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
            Camino=trajectories(movement_kind[pointer],rotation_step if pointer==3 else translation_step,Tactual,n)
            i=0
            while i<n:
                qs=q_invs(l, Camino[i])[1]
                move(qs)
            Tactual=Camino[n]
            # function(movement_kind[pointer],rotation_step if pointer==3 else (-translation_step)))
        if (letter==b'\x1b'):
            # Escape with crtl+z
            break