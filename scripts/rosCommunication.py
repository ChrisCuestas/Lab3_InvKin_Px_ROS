# ROS imports
import termios, sys, os
import numpy as np
import rospy
# from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
# from turtlesim.srv import Spawn

import time

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

def grad21024 (num):
    return 512+(num*(511/180))
def home():
    q =np.array([0,0,45,60])
    q1=np.array([grad21024(q[0]),grad21024(q[1]),grad21024(q[2]),grad21024(q[3])])
    jointCommand('', 6, 'Goal_Position', q1[0], 0.5)
    jointCommand('', 7, 'Goal_Position', q1[1], 0.5)
    jointCommand('', 8, 'Goal_Position', q1[2], 0.5)
    jointCommand('', 9, 'Goal_Position', q1[3], 0.5)
    time.sleep(0.5)
def move(q):
    q1=np.array([grad21024(q[0]),grad21024(q[1]),grad21024(q[2]),grad21024(q[3])])
    jointCommand('', 6, 'Goal_Position', q1[0], 0.5)
    jointCommand('', 7, 'Goal_Position', q1[1], 0.5)
    jointCommand('', 8, 'Goal_Position', q1[2], 0.5)
    jointCommand('', 9, 'Goal_Position', q1[3], 0.5)

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