# -*- coding: utf-8 -*-
"""
Created on Mon Oct 21 15:42:11 2019

@author: Muhammad
"""
import numpy as np
from math import cos, sin, acos
import modern_robotics as mr
from scipy.linalg import pinv

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr, accumulated_error):
    iX_Xd = np.matmul(np.linalg.inv(X), Xd)
    
    Adj_iX_Xd = mr.Adjoint(iX_Xd)
    Vd = mr.se3ToVec(1/dt*(np.matmul(np.linalg.inv(Xd),Xd_next)))
    
    Xerr = mr.se3ToVec(mr.MatrixLog6((np.matmul(np.linalg.inv(X),Xd))))
    #print(Xerr)
#    accumulated_error = Xerr+accumulated_error
    #print(accumulated_error)
    feedback_error = np.matmul(Kp,Xerr)+np.matmul(Ki, accumulated_error)
#    print("AE ",(np.matmul(Kp,Xerr)).shape)
#    print("FE ", (np.matmul(Ki, accumulated_error*dt)).shape)
    V = np.matmul(Adj_iX_Xd, Vd)+feedback_error
#    V = feedback_error
    return V

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    z_standoff = Tce_standoff[2][3]
    #Segment 1
    z_base = Tsc_initial[2][3]
    #print(Tsc_initial)
    #print(z_standoff)
    Tsc1_standoff = np.copy(Tsc_initial)
    Tsc1_standoff[2][3]=z_base+z_standoff
    #print(Tsc1_standoff)
    Tse1_standoff = np.matmul(Tsc1_standoff,Tce_standoff)
    #print(Tsc_initial)
    Xstart1 = Tse_initial
    Xend1 = Tse1_standoff
    #Segment 2
    Tse1_grasp = np.matmul(Tsc_initial,Tce_grasp)
    Xstart2 = Tse1_standoff
    Xend2 = Tse1_grasp
    #print("Tsc_initial = ",Tsc_initial)
    #print("Tse1_grasp = ",Xend2)
    #segment 3
    Xstart3 = Tse1_grasp
    Xend3 = Tse1_grasp
    #segment 4
    Xstart4 = Tse1_grasp
    Xend4 = Tse1_standoff    
    #Segment 5
    Tsc2_standoff = np.copy(Tsc_final)
    Tsc2_standoff[2][3]=z_base + z_standoff
    Tse2_standoff = np.matmul(Tsc2_standoff,Tce_standoff)
    
    Xstart5 = Tse1_standoff  
    Xend5 = Tse2_standoff
    #Segment 6
    Tse2_grasp = np.matmul(Tsc_final,Tce_grasp)
    Xstart6 = Tse2_standoff  
    Xend6 = Tse2_grasp
    #Segment 7
    Xstart7 = Tse2_grasp
    Xend7 = Tse2_grasp     
    #Segment 8
    Xstart8 = Tse2_grasp
    Xend8 = Tse2_standoff
    
    #Screw Trajectory Calculation for 1-8 segments

    Tf = k
    T1 = 4
    T2 = 2
    T3 = 1
    T4 = T2
    T5 = T1
    T6 = T2
    T7 = T3
    T8 = T2
    N1 = T1*Tf/0.01
    N2 = T2*Tf/0.01
    N3 = T3*Tf/0.01
    N4 = T4*Tf/0.01
    N5 = T5*Tf/0.01
    N6 = T6*Tf/0.01
    N7 = T7*Tf/0.01
    N8 = T8*Tf/0.01
    
    method = 5
    lst = []
    g_state = 0
    output1 = mr.ScrewTrajectory(Xstart1, Xend1, Tf, N1, method)
    lst = create_lst(output1,  lst, g_state)
    g_state = 0    
    output2 = mr.ScrewTrajectory(Xstart2, Xend2, Tf, N2, method)
    lst = create_lst(output2, lst, g_state)
    g_state = 1
    output3 = mr.ScrewTrajectory(Xstart3, Xend3, Tf, N3, method)
    lst = create_lst(output3,  lst, g_state)
    g_state = 1
    output4 = mr.ScrewTrajectory(Xstart4, Xend4, Tf, N4, method)
    lst = create_lst(output4,  lst, g_state)
    g_state = 1
    output5 = mr.ScrewTrajectory(Xstart5, Xend5, Tf, N5, method)
    lst = create_lst(output5, lst, g_state)
    g_state = 1
    output6 = mr.ScrewTrajectory(Xstart6, Xend6, Tf, N6, method)
    lst = create_lst(output6,  lst, g_state)
    g_state = 1
    output7 = mr.ScrewTrajectory(Xstart7, Xend7, Tf, N7, method)
    lst = create_lst(output7,  lst,g_state)
    g_state = 0
    output8 = mr.ScrewTrajectory(Xstart8, Xend8, Tf, N8, method)
    lst = create_lst(output8,  lst,g_state)
#    f = open("output.csv", "w") 
#    lst = [output1, output2, output3, output4, output5, output6, output7, output8]
#    num = 1
#    for out in lst:
#        if num >= 3 and num <= 7:
#            gripper_state = 1
#        else:
#            gripper_state = 0
#        for i in out:
#            write_csv(i,gripper_state, f)
#        num=num+1
#    f.close()

    #output = output1+output2+output3+output4+output5+output6+output7+output8
    #print(lst)
    #print(len(lst))
    return lst

def create_lst(np_array,elst, gs):
    j = 0
    for i in np_array:
        r11 = i[0][0]
        r12 = i[0][1]
        r13 = i[0][2]
    
        r21 = i[1][0]
        r22 = i[1][1]
        r23 = i[1][2]
    
        r31 = i[2][0]
        r32 = i[2][1]
        r33 = i[2][2]
    
        px = i[0][3]
        py = i[1][3]
        pz = i[2][3] 
        
        elst.append([r11 ,r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gs])
        j = j+1
    return elst


def NextState(chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4 \
             ,gripper_state, J_speed1, J_speed2, J_speed3, J_speed4, J_speed5, u1,u2,u3,u4, dt, c):
    
    r = 0.0475
    w = 0.3/2
    l = 0.47/2
    
    new_J1 = J1 + J_speed1*dt
    new_J2 = J2 + J_speed2*dt
    new_J3 = J3 + J_speed3*dt
    new_J4 = J4 + J_speed4*dt
    new_J5 = J5 + J_speed5*dt
    
    new_W1 = W1 + u1*dt
    new_W2 = W2 + u2*dt
    new_W3 = W3 + u3*dt
    new_W4 = W4 + u4*dt
    u = np.mat([[u1],[u2],[u3],[u4]])
    F = r/4*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
             [1.0,        1.0,           1.0,           1.0],
             [-1.0,        1.0,         -1.0,           1.0]])   
    
    Vb = np.matmul(F,u)
    #Vb6 = [0.0,0.0,np.asscalar(Vb[0]),np.asscalar(Vb[1]),np.asscalar(Vb[2]),0.0]
    #print(Vb6)
    Vb6 = np.array([0, 0, np.asscalar(Vb[0]), np.asscalar(Vb[1]), np.asscalar(Vb[2]), 0])
    _Vb6_ = mr.VecTose3(Vb6)

#    _Vb6_ = np.array([[0,                     np.asscalar(-Vb[0]), 0,    np.asscalar(Vb[1])],
#                      [np.asscalar(Vb[0]),   0,                  0,    np.asscalar(Vb[2])],
#                      [0,                    0,                  0,    0],
#                      [0,                    0,                  0,    0]])
    #print("[Vb6] =",_Vb6_)
#    _Vb_ = [[0.0,                  np.asscalar(-Vb[2]),     np.asscalar(Vb[1])],
#                     [np.asscalar(Vb[2]),    0.0,                     np.asscalar(Vb[0])],
#                     [np.asscalar(Vb[1]),    np.asscalar(Vb[0]),                    0.0]]
#    
#    
#    print((_Vb_))
#    
#    _Vb6_ = np.array([np.zeros(3), np.zeros(3), _Vb_[0], _Vb_[1], _Vb_[2], np.zeros(3)])
#    print("\n",_Vb6_)
    Tsb = np.array([[cos(chassis_phi), -sin(chassis_phi),   0,  chassis_x],
                    [sin(chassis_phi), cos(chassis_phi),   0,   chassis_y],
                    [0,                0,                  1,        0.0963],
                    [0,                0,                  0,        1]])
    #print("\nTsb =",Tsb)
    Tb1 = mr.MatrixExp6(_Vb6_*dt)
    #print("\nTb1 =", Tb1)
    Tsb1 = np.matmul(Tsb,Tb1)
    #print("\nTsb1 =",Tsb1,Tsb1[0][0])
    
    new_chassis_phi = acos(Tsb1[0][0])
    new_chassis_x = Tsb1[0][3]
    new_chassis_y = Tsb1[1][3]
#    print("\nchassis_x = ",new_chassis_x)
#    print("\nchassis_y = ",new_chassis_y)
    #print("\nchassis_phi = ",chassis_phi)
    return new_chassis_phi, new_chassis_x, new_chassis_y, new_J1, new_J2, new_J3, new_J4, new_J5, new_W1, new_W2, new_W3, new_W4


""" Home configuration matrices"""
chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4 = 0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

M0e = np.array([[1, 0, 0,     0.033],
                [0, 1, 0,         0],
                [0, 0, 1,    0.6546],
                [0, 0, 0,         1]])

Tb0 = np.array([[1, 0, 0,     0.1662],
                [0, 1, 0,     0],
                [0, 0, 1, 0.0026],
                [0, 0, 0,     1]])
Tbe = np.matmul(Tb0, M0e)

Tsb = np.array([[cos(chassis_phi),-sin(chassis_phi),    0,    chassis_x],
                [sin(chassis_phi), cos(chassis_phi),    0,    chassis_y],
                [0,                 0,                  1,      0.0963],
                [0,                 0,                  0,          1]])

""" Reference Trajectory Generation"""
z_standoff = 0.06
y_rotation = 145*np.pi/180.0

Tce1_standoff =   np.array([[cos(y_rotation),   0,            sin(y_rotation),           0],
                            [0,                 1,            0,                         0],
                            [-sin(y_rotation),  0,           cos(y_rotation),    z_standoff],
                            [0,                 0,            0,                          1]])

Tse_home = np.matmul(np.matmul(Tsb,Tb0),M0e)

Tsc_home =    np.array([[1, 0, 0,     1.025],
                        [0, 1, 0,     0.45],
                        [0, 0, 1, 0.025],
                        [0, 0, 0,     1]])

Tce1_grasp =      np.array([[cos(y_rotation),      0,          sin(y_rotation),     0],
                            [0,                 1,             0,                   0],
                            [-sin(y_rotation),  0,             cos(y_rotation),     0],
                            [0,                 0,             0,                   1]])



Tsc_goal =      np.array([[0, 1,  0,     4.7510e-01],
                          [-1, 0, 0,    -1.0750e+00],
                          [0, 0,  1, 0.025],
                          [0, 0,  0,     1]])
    

B1, B2, B3, B4, B5 = [0, 0, 1,   0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0],[0, -1, 0, -0.2176, 0, 0], [0, 0, 1,   0,      0, 0]

Blist = np.array(([B1, B2, B3, B4, B5])).T

#print(Blist)

_B1_ = mr.VecTose3([0, 0, 1,   0, 0.033, 0])
_B2_ = mr.VecTose3([0, -1, 0, -0.5076, 0, 0])
_B3_ = mr.VecTose3([0, -1, 0, -0.3526, 0, 0])
_B4_ = mr.VecTose3([0, -1, 0, -0.2176, 0, 0])
_B5_ = mr.VecTose3([0, 0, 1,   0,      0, 0])

r = 0.0475
w = 0.3/2
l = 0.47/2

F = r/4*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
        [1.0,        1.0,           1.0,           1.0],
        [-1.0,        1.0,         -1.0,           1.0]])  



# npad is a tuple of (n_before, n_after) for each dimension
npad = ((2, 1), (0, 0))
F6 = np.pad(F, pad_width=npad, mode='constant', constant_values=0)

dt = 0.01    
k = 1
c = 100
Xerr_lst = []
acc_error = np.zeros((6))
#print(acc_error)
N_configurations = TrajectoryGenerator(Tse_home, Tsc_home, Tsc_goal, Tce1_grasp, Tce1_standoff, k)

#print(".....................................",N_configurations)
f = open("output.csv", "w") 
e = open("error.csv", "w") 
""" Reference Trajectory utlization"""
t_int = 0
for i in range(len(N_configurations)-1):
    Xref_lst = np.array(N_configurations[i])
    Xref_next_lst = np.array(N_configurations[i+1])
    
    
    r11 = Xref_lst[0]
    r12 = Xref_lst[1]
    r13 = Xref_lst[2]
    
    r21 = Xref_lst[3]
    r22 = Xref_lst[4]
    r23 = Xref_lst[5]
    
    r31 = Xref_lst[6]
    r32 = Xref_lst[7]
    r33 = Xref_lst[8]
    
    px = Xref_lst[9]
    py = Xref_lst[10]
    pz = Xref_lst[11]
    
    gripper_state = Xref_lst[12]
    csv_output = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %d \n" % \
        (chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper_state)
    f.write(csv_output)
    Xref = np.array([[r11, r12, r13, px],
                     [r21, r22, r23, py],
                     [r31, r32, r33, pz],
                     [0,   0,   0,   1]])
    
    r11 = Xref_next_lst[0]
    r12 = Xref_next_lst[1]
    r13 = Xref_next_lst[2]
    
    r21 = Xref_next_lst[3]
    r22 = Xref_next_lst[4]
    r23 = Xref_next_lst[5]
    
    r31 = Xref_next_lst[6]
    r32 = Xref_next_lst[7]
    r33 = Xref_next_lst[8]
    
    px = Xref_next_lst[9]
    py = Xref_next_lst[10]
    pz = Xref_next_lst[11]
    
    Xref_next = np.array([[r11, r12, r13, px],
                          [r21, r22, r23, py],
                          [r31, r32, r33, pz],
                          [0,   0,   0,   1]])
    
    thetaBlist = np.array(([J1, J2, J3, J4, J5]))
    
    Tsb = np.array([[cos(chassis_phi),-sin(chassis_phi),    0,    chassis_x],
                    [sin(chassis_phi), cos(chassis_phi),    0,    chassis_y],
                    [0,                 0,                  1,      0.0963],
                    [0,                 0,                  0,          1]])
    
    Tse_home = np.matmul(Tsb,Tbe)

    Xse =      mr.FKinBody(Tse_home, Blist, thetaBlist)
    
    X0e =      mr.FKinBody(M0e, Blist, thetaBlist)

    Jarm = mr.JacobianBody(Blist, thetaBlist)
    
    Jbase = mr.Adjoint(np.linalg.inv(X0e) @ np.linalg.inv(Tb0)) @ F6
    Je = np.hstack((Jbase, Jarm))
    iJe = pinv(Je,3e-3)
    
    Ki = 0.001*np.eye(6)
    Kp = 2*np.eye(6)
    
    Xerr = mr.se3ToVec(mr.MatrixLog6((np.matmul(np.linalg.inv(Xse),Xref))))
    acc_error = acc_error+ Xerr*dt
    V = FeedbackControl(Xse, Xref, Xref_next, Kp, Ki, dt, Xerr, acc_error)
    Xerr_lst = Xerr
    
    
    csv_output = "%10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f \n" % (t_int, Xerr_lst[0], Xerr_lst[1], Xerr_lst[2], Xerr_lst[3],Xerr_lst[4], Xerr_lst[5])
    t_int = t_int + 0.01
    e.write(csv_output)
#    print(Xerr.shape)
#    print(iJe.shape)
#    print(V.shape)
    output = np.matmul(iJe, V)
    uw1, uw2, uw3, uw4, js1, js2, js3, js4, js5 = output[0], output[1], output[2], output[3], output[4]\
                                                 ,output[5], output[6],output[7],output[8]
    
    #print(uw1,uw2,uw3,uw4)
    newchassis_phi, newchassis_x, newchassis_y, newJ1, newJ2, newJ3, newJ4, newJ5, newW1, newW2, newW3, newW4 \
    = NextState(chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4 \
              ,gripper_state, js1, js2, js3, js4, js5, uw1,uw2,uw3,uw4, dt, c)
    chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4 = newchassis_phi, newchassis_x, newchassis_y, newJ1, newJ2, newJ3, newJ4, newJ5, newW1, newW2, newW3, newW4
    
f.close()
e.close()

