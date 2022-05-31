from sympy.matrices import *
import numpy as np
    # def q_inv(T,l):
    #     #a = T(1:3,3)
    #     a= T([T(1,3)],[T(2,3)],[T(3,3)])
    #     x = T(1,4)
    #     y = T(2,4)
    #     z = T(3,4)
    # # Wrist
    #     w = [x y z] -l(4)*a
if __name__ == "__main__":
    l=np.array([1, 2, 3, 4])
    T=np.matrix([[1, 2, 3, 4],[5, 6, 7, 8],[9, 10, 11, 12],[13, 14, 15, 16]])
    Pw = T[0:3, 3]-(l[3]*T[0:3, 2])

    q1a=np.artan2(T[1,3],T[0,3])
    q1b=np.artan2(-T[1,3],-T[0,3])
    pxy = np.sqrt(Pw[0]^2 + Pw[1]^2)
    z = Pw[4] - l[0]
    r = np.sqrt(pxy^2 + z^2)
    
    
