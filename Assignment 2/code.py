import modern_robotics as mr
import numpy as np
import sys
import re

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 00, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    csv=[]                                                                                                          
    csv.append(list(thetalist0))                                                                                        #To create a seperate matrix which will
    maxiterations = 20                                                                                                  #save the values of joint vectors of each iteration
    np.set_printoptions(precision=4)                                                                                     
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,thetalist)), T)))                    
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        print("Iteration {}:".format(i))
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist,thetalist)), Vb)
        print("Joint Vector:")
        thetalist1=['%.5f' % t for t in list(thetalist)]                                                                #To print Vb without brackets and rounded off to 5 digits
        print(*thetalist1,sep=', ')
        print()
        i = i + 1
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,thetalist)), T)))
        print("SE(3) end−effector config:")
        print(re.sub('[\[\]]', '', np.array_str(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,thetalist)), T)))))   #To print the SE(3) end−effector configuration
        print()                                                                                                             #without brackets 
        print("error twist V_b:")       
        Vb1 =['%.5f' % b  for b in list(Vb)]
        print(*Vb1,sep=', ')                                                                                        #To print Vb without brackets and rounded off to 5 digits
        print()
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        print("angular error magnitude ||omega_b||: ",np.linalg.norm([Vb[0], Vb[1], Vb[2]]))                        #To print the angular error magnitude
        print()
        csv.append(list(thetalist))
        print("linear error magnitude ||v_b||: ",np.linalg.norm([Vb[3], Vb[4], Vb[5]]))                             #To print the linear error magnitude
        print()
    
    np.savetxt("iterates.csv",csv,delimiter=',')                                                                    #To save csv array as a .csv file
    return (thetalist, not err)

Blist = np.array([[0,1,0,0.191,0,0.817],
                  [0,0,1,0.095,-0.817,0],
                  [0,0,1,0.095,-0.392,0],
                  [0,0,1,0.095,0,0],
                  [0,-1,0,-0.082,0,0],
                  [0,0,1,0,0,0]]).T
M = np.array([[-1,0,0,0.817],
              [0,0,1,0.191],
              [0,1,0,0.006],
              [0,0,0,1]])
T = np.array([[0,1,0,-0.5],
              [0,0,-1,0.1],
              [-1,0,0,0.1],
              [0,0,0,1]])
thetalist0 = np.array([2.72761, 1.03152, -2.52556, 0.38711, -0.37419, -0.46378])
print("Initial guess:")
print(*thetalist0,sep=', ')
eomg = 0.001
ev = 0.0001
[thetalist,success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
print("Thetalist:")
print(*thetalist,sep=', ')
print("Success: ",success)
