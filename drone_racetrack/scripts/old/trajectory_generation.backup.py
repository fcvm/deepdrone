# -*- coding: utf-8 -*-


import numpy as np
import scipy.linalg

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Quadratic Programming in Python
# https://scaron.info/blog/quadratic-programming-in-python.html
# import quadprog
import qpsolvers
import cvxopt


import gurobipy





def get_snap_trajectory(keyframes, t_keyframes):


    # Number of keyframes
    m = keyframes.shape[0]



    # *** PARAMETERS ***


    # Order of derivative of position squared, to be minimized
    k_r = 4 # Snap

    # Order of derivative of yaw angle squared, to be minimized
    k_psi = 4 #k_psi = 2 in paper

    # Constant that makes the position-integrand nondimensional
    mu_r = 1

    # constant that makes the yaw-angle-integrand nondimensional
    mu_psi = 1

    # Order of the polynomial basis functions for trajectory
    # e.g. - n = 3, then  1 + x + x²
    # SHOULD BE BIGGER THAN k_r !!!!!!!!!!!!!!
    n=19

    # *** \PARAMETERS ***




    # *** MODEL QP ***

    # min     (1/2) x^T P x + q^T x
    # s.t.    Gx + s = h
    #         Ax = b
    #         s >= 0


    # 4 independent QP for x, y, z, phi

    xyzphi =	{
        "x"     :   [k_r],
        "y"     :   [k_r],
        "z"     :   [k_r],
        "phi"   :   [k_psi]
    }


    # Derivative Matrices
    D = np.zeros((n, n))
    for i in range(1, n):
        D[i - 1, i] = i
    # k_r-th derivative matrix
    D_r = np.linalg.matrix_power(D, k_r)
    # k_psi-th derivative matrix
    D_psi = np.linalg.matrix_power(D, k_psi)


    # P
    P_r = mu_r * scipy.linalg.block_diag(*([np.matmul(D_r.T, D_r)] * (m - 1)))
    P_psi = mu_psi * scipy.linalg.block_diag(*([np.matmul(D_psi.T, D_psi)] * (m - 1)))


    xyzphi['x'].append(P_r)
    xyzphi['y'].append(P_r)
    xyzphi['z'].append(P_r)
    xyzphi['phi'].append(P_psi)


    # Polynomial powered t_keyframes array
    T_keyframes = t_keyframes ** np.arange(n)[np.newaxis, :]


    for qp in xyzphi:

        if qp == 'x':
            whichQP = 0
        elif qp == 'y':
            whichQP = 1
        elif qp == 'z':
            whichQP = 2
        elif qp == 'phi':
            whichQP = 3


        # A and b
        A_start = np.zeros((m - 1, n * (m - 1)))
        A_stop = np.zeros((m - 1, n * (m - 1)))
        b_start = np.zeros((m - 1, 1))
        b_stop = np.zeros((m - 1, 1))

        for i in range(m - 1):
            
            A_start[i, i * n : i * n + n] = T_keyframes[i, :][np.newaxis, :]
            #print(T_keyframes[i, :][np.newaxis, :])
            A_stop[i, i * n : i * n + n] = T_keyframes[i + 1, :][np.newaxis, :]
            #print(T_keyframes[i + 1, :][np.newaxis, :])

            b_start[i, 0] = keyframes[i, whichQP]
            b_stop[i, 0] = keyframes[i + 1, whichQP]


        A = np.vstack((A_start, A_stop))
        b = np.vstack((b_start, b_stop))


        for j in range((xyzphi[qp][0] - 2) * (m - 1)):

            A_dev = np.zeros((m - 1, (m - 1) * n))
            b_dev = np.zeros((m - 1, 1))


            for i in range(m - 2):

                A_dev[i, i * n : i * n + n] = np.matmul(T_keyframes[i + 1, :][np.newaxis, :], np.linalg.matrix_power(D, j + 1))
                A_dev[i, i * n + n : i * n + n + n] = - np.matmul(T_keyframes[i + 1, :][np.newaxis, :], np.linalg.matrix_power(D, j + 1))

            A_dev[-1:, 0 : n] -= np.matmul(T_keyframes[0, :][np.newaxis, :], np.linalg.matrix_power(D, j + 1))
            A_dev[-1:, (m - 2) * n : (m - 1) * n] += np.matmul(T_keyframes[-1, :][np.newaxis, :], np.linalg.matrix_power(D, j + 1))


            A = np.vstack((A, A_dev))
            b = np.vstack((b, b_dev))


        # Test model
        qp_p = A.shape[0]
        qp_n = A.shape[1]

        rank_A = np.linalg.matrix_rank(A)
        rank_PA = np.linalg.matrix_rank(np.vstack((P_r, A)))

        if not rank_A == qp_p:
            print('For ', qp, '-QP ', '!!! rank(A) = ', rank_A, 'not equal #{rows of A} = ', qp_p, ' !!!')
        if not rank_PA == qp_n:
            print('For ', qp, '-QP ', '!!! rank([P,G,A]^T) = ', rank_PA, 'not equal #{columns of G or A} = ', qp_n, ' !!!')

        
        xyzphi[qp].append(A)
        xyzphi[qp].append(b)

    # *** \MODEL QP ***


    # *** SOLVE QP ***

    #c_opt = quadprog_solve_qp_equality(H, b[:, 0], G=A, h=b[:, 0])[0]
    #c_opt = quadprog_solve_qp(H, b, A=A, b=b)
    #c_opt = qpsolvers.solve_qp(H, b[:, 0], G=A*0, h=b[:, 0]*0, A=A, b=b[:, 0], solver='quadprog')
    #c_opt = qpsolvers.solve_qp(H, b[:, 0], G=A, h=b[:, 0]+1, solver='quadprog')
    #c_opt = qpsolvers.solve_qp(H, b[:, 0], solver='quadprog')
    #c_opt = quadprog.solve_qp(H, b[:, 0])

    for qp in xyzphi:

        P = cvxopt.matrix(xyzphi[qp][1], tc='d')
        q = cvxopt.matrix(np.zeros((n * (m - 1),)), tc='d')
        G = None
        #G = cvxopt.matrix(- np.eye(n * (m - 1)), tc='d')
        h = None
        #h = cvxopt.matrix(np.zeros((n * (m - 1),)), tc='d')
        A = cvxopt.matrix(xyzphi[qp][2], tc='d')
        print(np.size(A))
        b = cvxopt.matrix(xyzphi[qp][3][:, 0], tc='d')

        sol = cvxopt.solvers.qp(P, q, G=G, h=h, A=A, b=b)

        #P = np.array(P)
        #q = np.array(q)
        #G = - np.eye(n * (m - 1))
        #h = np.zeros((n * (m - 1),))
        #A = np.array(A)
        #b = np.array(b)

        #sol = qpsolvers.solve_qp(P, q, G, h, A, b)[:, np.newaxis]

        pol_coeffs = np.array(sol['x'])

        #print(pol_coeffs.shape)

        pol_coeffs = np.reshape(pol_coeffs, (m - 1, n)).T

        xyzphi[qp].append(pol_coeffs)

    # *** \SOLVE QP ***



    # *** TRAJECTORY ***

    for qp in xyzphi:

        # time step
        dt = 1e-3

        # allocation for vstack
        t = np.array([[ None ]], dtype=np.float64)
        traj = np.array([[ None ]], dtype=np.float64)


        for i in range(m - 1):

            samples = np.round((t_keyframes[i + 1] - t_keyframes[i]) / dt)

            tt = np.linspace(t_keyframes[i], t_keyframes[i + 1], num=samples, endpoint=False)[:, np.newaxis]

            TT = tt[:, np.newaxis] ** np.arange(n)[np.newaxis, :]

            trajec = np.matmul(TT, xyzphi[qp][4][:, i])

            t = np.vstack((t, tt))

            traj = np.vstack((traj, trajec))

        t = t[1:, :]
        traj = traj[1:, :]

        xyzphi[qp].append(t)
        xyzphi[qp].append(traj)
    
    t = xyzphi['x'][-2]
    traj_x = xyzphi['x'][-1]
    traj_y = xyzphi['y'][-1]
    traj_z = xyzphi['z'][-1]
    traj_phi = xyzphi['phi'][-1]

    # *** \TRAJECTORY ***

    return(t, traj_x, traj_y, traj_z, traj_phi)






if __name__ == "__main__":



    # *** INPUTS  ***


    # keyframe: [x, y, z, ψ]
    keyframes = np.array([  [0, 0, 0, 0 ],
                            [1.5, 1.5, .1, 45 ],
                            [3, 3, 0.2, 90 ],
                            [2, 4, 0.2, 135 ],
                            [0, 6, 0.15, 180 ],
                            [-3, 3, 0.13, 270],
                            [-2, -5, 0.1, 290],
                            [-0, -4, 0.07, 340],
                            [0, 0, 0, 0 ]        ],      dtype=np.float64)
    #keyframes = np.array([  [0, 0, 0, 0 / 180 * np.pi],
    #                        [3, 3, 0, 90 / 180 * np.pi],
    #                        [0, 6, 0, 180 / 180 * np.pi],
    #                        [-3, 3, 0, 270/ 180 * np.pi],
    #                        [0, 0, 0, 0 / 180 * np.pi]        ],      dtype=np.float64)
    #keyframes = np.array([  [0, 0, 0, 0],
    #                        [3, 3, 0, 90],
    #                        [0, 6, 0, 180]
    #                                            ],      dtype=np.float64)



    # Time points when being at keyframes 
    # !!! NUMERICAL ERRORS WHEN TO0 HIGH IN COMBINATION WITH TOO GREAT n
    t_keyframes = np.array([    [0], 
                                [1],
                                [2], 
                                [3], 
                                [4],
                                [5],
                                [6],
                                [7],
                                [8]        ],      dtype=np.float64)
    #t_keyframes = np.array([    [0], 
    #                            [1],
    #                            [2],
    #                                        ],      dtype=np.float64)

    # *** \INPUTS  ***



    t, traj_x, traj_y, traj_z, traj_phi = get_snap_trajectory(keyframes, t_keyframes)


    # *** PLOT ***

    t = np.squeeze(t)
    traj_x = np.squeeze(traj_x)
    traj_y = np.squeeze(traj_y)
    traj_z = np.squeeze(traj_z)
    traj_phi = np.squeeze(traj_phi)

    '''
    plt.figure()
    plt.title('X')
    plt.plot(t, traj_x)
    plt.show()


    plt.figure()
    plt.title('Y')
    plt.plot(t, traj_y)
    plt.show()


    plt.figure()
    plt.title('Z')
    plt.plot(t, traj_z)
    plt.show()


    plt.figure()
    plt.title('Phi')
    plt.plot(t, traj_phi)
    plt.show()


    plt.figure()
    plt.plot(traj_x, traj_y)
    plt.show()
    '''

    point  = np.array([0, 0, 0])
    normal = np.array([0, 0, 1])

    point2 = np.array([10, 50, 50])

    # a plane is a*x+b*y+c*z+d=0
    # [a,b,c] is the normal. Thus, we have to calculate
    # d and we're set
    d = -point.dot(normal)
    xx, yy = np.meshgrid(range(-10, 10), range(-10, 10))
    z = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(xx, yy, z, alpha=0.2)
    ax.plot(traj_x, traj_y, traj_z, label='parametric curve')
    ax.scatter(keyframes[:, 0], keyframes[:, 1], keyframes[:, 2], s=200)
    ax.legend()
    ax.set_zlim(-1,1)
    plt.show()
    #Axes3D.plot(np.squeeze(traj_x), np.squeeze(traj_y))


    # *** \PLOT ***

