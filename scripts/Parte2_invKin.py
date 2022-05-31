# Mecanismo 2R
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *


the3 = np.arccos((r**2 - l[2]**2 - l[3]**2)/(2*l[2]*l[3]))
if np.isreal(the3):
    alp = np.arctan2(z,pxy)
    the2a = alp - np.arctan2(l[3]*np.sin(the3),l[2]+l[3]*np.cos(the3))
    the2b = alp + np.arctan2(l[3]*np.sin(the3),l[2]+l[3]*np.cos(the3))

    # Codo abajo
    q2a = -np.pi/2 + the2a      # q2
    q3a = the3;                 # q3

    #Codo arriba
    q2b = -np.pi/2 + the2b      # q2
    q3b = -the3;                # q3

    # Orientacion
    R_p = ((rotz(q1a)).transpose)@T[1:3,1:3]
    pitch = np.arctan2(R_p[3,1],R_p[1,1])
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

q_inv[1,1:4] = np.array([q1a, q2a, q3a, q4a])
q_inv[2,1:4] = np.array([q1a, q2b, q3b, q4b])

# Queda invertido el yaw debido a la restriccion de movimiento
# Usando q1b
q_inv[3,1:4] = np.array([q1b, -q2a, -q3a, -q4a])
q_inv[4,1:4] = np.array([q1b, -q2b, -q3b, -q4b])

for i in len(q_inv[1,:]):
    if np.any(np.isnan(q_inv[i,:])):
        q_inv[i,:] = np.array([np.NaN,np.NaN,np.NaN,np.NaN])

