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
    
def q_invs(l, T):
    # l=np.array([1, 2, 3, 4])
    # T=np.matrix([[1, 2, 3, 4],[5, 6, 7, 8],[9, 10, 11, 12],[13, 14, 15, 16]])
    Pw = T[0:3, 3]-(float(l[3])*T[0:3, 2])

    # print(Pw)

    q1a=float(np.arctan2(T[1,3],T[0,3]))
    q1b=float(np.arctan2(-T[1,3],-T[0,3]))
    pxy = np.sqrt(float(Pw[0])**2 + float(Pw[1])**2)
    z = float(Pw[2]) - float(l[0])
    r = np.sqrt(pxy**2 + z**2)

    the3 = np.arccos((r**2 - float(l[1])**2 - float(l[2])**2)/(2*float(l[1])*float(l[2])))
    if np.isreal(the3):
        alp = np.arctan2(z,pxy)
        the2a = alp - np.arctan2(float(l[2])*np.sin(the3),float(l[1])+float(l[2])*np.cos(the3))
        the2b = alp + np.arctan2(float(l[2])*np.sin(the3),l[1]+float(l[2])*np.cos(the3))

        # Codo abajo
        q2a = -np.pi/2 + the2a      # q2
        q3a = the3;                 # q3

        #Codo arriba
        q2b = -np.pi/2 + the2b      # q2
        q3b = -the3;                # q3

        # Orientacion
        
        R_p = np.transpose(rotz(q1a))@T[0:3,0:3]
        pitch = np.arctan2(float(R_p[2,0]),float(R_p[0,0]))
        # Codo abajo
        q4a = pitch - (q2a + q3a)   # q4
        # Codo arriba
        q4b = pitch - (q2b + q3b)   # q4
    else:
        q2a = np.NaN
        q2b = np.NaN
        q3a = np.NaN
        q3b = np.NaN
        q4a = np.NaN
        q4b = np.NaN

    q_inv=np.array([[q1a, q2a, q3a, q4a],
                    [q1a, q2b, q3b, q4b],
                    [q1b, -q2a, -q3a, -q4a],
                    [q1b, -q2b, -q3b, -q4b]],dtype=object)
    # print("q_inv")
    # print(q_inv)
    for i in range(4):
        for o in q_inv[i,:]:
            if np.isnan(o):
                q_inv[i,:] = np.array([np.NaN,np.NaN,np.NaN,np.NaN])
    return (q_inv)


def grad21024 (num):
    return 512+(num*(511/180))

def home():
    q =np.array([0,0,45,60])
    q1=np.array([grad21024(q[0]),grad21024(q[1]),grad21024(q[2]),grad21024(q[3])])
    # jointCommand('', 6, 'Goal_Position', q1[0], 0.5)
    # jointCommand('', 7, 'Goal_Position', q1[1], 0.5)
    # jointCommand('', 8, 'Goal_Position', q1[2], 0.5)
    # jointCommand('', 9, 'Goal_Position', q1[3], 0.5)
    # time.sleep(0.5)

if __name__ == "__main__":
    q =np.array([90,30,30,30])
    # home(q)
    l = np.array([14.5, 10.7, 10.7, 9])
    qlims = np.array([[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4]])
    robot = rtb.DHRobot(
    [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
    rtb.RevoluteDH(qlim=qlims[0,:])],
    name="Px_DH_std")
    # robot.tool = transl(l[3],0,0).dot(troty(np.pi/2).dot(trotz(-np.pi/2)))
    robot.tool=np.matrix([[0, 0, 1 ,int(l[3])],[-1, 0, 0, 0],[0, -1, 0, 0],[0, 0, 0, 1]])
    # print(robot)
    qt = np.deg2rad(q)
    print("Original q")
    print(qt)
    Tt = robot.fkine(qt)
    T_actual=np.array(Tt)
    print("T_actual")
    print(T_actual)
    print("InvKin de T_actual")
    print((q_invs(l, np.matrix(T_actual)))[1])

    translation_step = 15
    n = 10
    l = np.array([14.5, 10.7, 10.7, 9])

    # T_actual = np.array([[-0.1736, 0, 0.9848, 19.49],
    #                       [0,1,0, 0],
    #                       [-0,9848, 0, -0.1736, 29.61],
    #                       [0,0,0,1]])
    
    path = trajectories("traz",translation_step,SE3(T_actual),n)
    print("Camino:")
    print(path)
    # Calculating the inv kinematics of each pose and sending the result vector to the robot
    i = 0
    while i<n:
        print("Iteración:", i)
        qs = (q_invs(l, np.matrix(path[i])))[1]
        print("Variables de articulación")
        print(qs)
        
        qs = np.array([np.rad2deg(qs[0]),np.rad2deg(qs[1]),np.rad2deg(qs[2]),np.rad2deg(qs[3])])

        # print("Variables de articulación")
        print(qs)
        i = i + 1
    # Set the new and pose as the actual pose
    T_actual = path[-1]