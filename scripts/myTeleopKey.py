import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from turtlesim.srv import Spawn
import termios, sys, os
import numpy as np
from numpy import pi 
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *

TERMIOS = termios
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

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

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

def toCenter():
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        telA=rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp=telA(5,5,1.5)

    except rospy.ServiceException:
        pass

def giro180():
    rospy.wait_for_service('/turtle1/teleport_relative')
    try:
        telR=rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
        rel=telR(0,3.1416)
    except rospy.ServiceException:
        pass

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

# if __name__ == "__main__":
#     translation_step = 1.0
#     rotation_step = 0.01
#     movement_kind = np.array(['trax','tray','traz','rot'])
#     pointer = 0

#     while 1:
#         letter = getkey()
#         if(letter ==b'w' or letter == b'W'):
#             # Change kind of movement forwards with W key
#             pointer = (pointer + 1)%4
#             print('Changed to {}'.format(movement_kind[pointer]))
#         elif(letter ==b's' or letter == b'S'):
#             # Change kind of movement backwards with S key
#             pointer = (pointer - 1) if pointer>0 else 3
#             print('Changed to {}'.format(movement_kind[pointer]))
#         elif(letter ==b'd' or letter == b'D'):
#             # Do the selected movement in the positive direction with D key
#             print('{} +{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
#             # function(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
#         elif(letter ==b'a' or letter == b'A'):
#             # Do the selected movement in the negative direction with A key
#             print('{} -{}'.format(movement_kind[pointer],rotation_step if pointer==3 else translation_step))
#             # function(movement_kind[pointer],rotation_step if pointer==3 else (-translation_step)))
#         if (letter==b'\x1b'):
#             # Escape with crtl+z
#             break