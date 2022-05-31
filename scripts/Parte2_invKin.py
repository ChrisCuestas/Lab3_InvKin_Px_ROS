# Mecanismo 2R
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *

l=np.array([1, 2, 3, 4])
T=np.matrix([[1, 2, 3, 4],[5, 6, 7, 8],[9, 10, 11, 12],[13, 14, 15, 16]])
Pw = T[0:3, 3]-(l[3]*T[0:3, 2])

q1a=np.artan2(T[1,3],T[0,3])
q1b=np.artan2(-T[1,3],-T[0,3])
pxy = np.sqrt(Pw[0]^2 + Pw[1]^2)
z = Pw[4] - l[0]
r = np.sqrt(pxy^2 + z^2)

the3 = np.arccos((r**2 - l[1]**2 - l[2]**2)/(2*l[1]*l[2]))
if np.isreal(the3):
    alp = np.arctan2(z,pxy)
    the2a = alp - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    the2b = alp + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))

    # Codo abajo
    q2a = -np.pi/2 + the2a      # q2
    q3a = the3;                 # q3

    #Codo arriba
    q2b = -np.pi/2 + the2b      # q2
    q3b = -the3;                # q3

    # Orientacion
    R_p = ((rotz(q1a)).transpose)@T[0:3,0:3]
    pitch = np.arctan2(R_p[2,0],R_p[0,0])
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

q_inv=np.zeros(4)
q_inv[0,0:4] = np.array([q1a, q2a, q3a, q4a])
q_inv[1,0:4] = np.array([q1a, q2b, q3b, q4b])

# Queda invertido el yaw debido a la restriccion de movimiento
# Usando q1b
q_inv[2,0:4] = np.array([q1b, -q2a, -q3a, -q4a])
q_inv[3,0:4] = np.array([q1b, -q2b, -q3b, -q4b])

for i in len(q_inv[0,:]):
    if np.any(np.isnan(q_inv[i,:])):
        q_inv[i,:] = np.array([np.NaN,np.NaN,np.NaN,np.NaN])

