#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys


############################
'''
TO DO
-----
- GUROBI [x]
- GAZEBO INTEGRATION [x]
        Import Error: Gurobi
'''
############################



import numpy as np
import scipy.linalg

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

# Quadratic Programming in Python
# https://scaron.info/blog/quadratic-programming-in-python.html
import cvxopt

# Quadratic Programming with Gurobi
import gurobipy

def Gurobi_SolveQPMatrixNotation(obj_mat, obj_vec=None, ine_mat=None, ine_vec=None, equ_mat=None, equ_vec=None, sense=gurobipy.GRB.MINIMIZE, lb=-gurobipy.GRB.INFINITY, ub=gurobipy.GRB.INFINITY, vtype=gurobipy.GRB.CONTINUOUS):
    # This function solves QP models in matrix notation:
    #  min/max
    #      dec_vec^T * obj_mat * dec_vec + obj_vec^T * dec_vec
    #  subject to
    #      ine_mat * dec_vec <= ine_vec
    #      equ_mat * dec_vec  = equ_vec
    #      x, y, z continuous/binary/non-negative integer/...
    #
    # Arguments of the function should be numpy.arrays

    #if none - > ZERO VECTORS!!!!!!!!!!!!!!


    ### CREATE MODEL ###
    m = gurobipy.Model("qp")

    ### CREATE DECISION VARIABLES ###
    # Number of decision variables in dec_vec
    n_dec = obj_mat.shape[0]
    # Declare variables
    dec_vec = m.addVars( 
        n_dec,
        lb=lb, 
        ub=ub,
        vtype=vtype
    ) 
    
    ### SET OBJECTIVE ###
    if obj_vec is None:
        obj_vec = np.zeros( (n_dec, ) )
    obj = 0
    for i in range(n_dec): 
        obj_mat_row = obj_mat[i, :]
        obj += obj_vec[i] * dec_vec[i]
        for j in range(n_dec):
            obj += dec_vec[i] * obj_mat_row[j] * dec_vec[j]
    m.setObjective(obj, sense=sense)

    ### SET CONSTRAINS
    # Equality
    if equ_mat is not None:
        for i in range(equ_mat.shape[0]):
            equ_rhs = equ_vec[i]
            equ_lhs = 0
            for j in range(n_dec):
                equ_lhs += equ_mat[i, j] * dec_vec[j]
            m.addConstr(equ_lhs == equ_rhs)
    # Inequality
    if ine_mat is not None:
        for i in range(ine_mat.shape[0]):
            ine_rhs = ine_vec[i]
            ine_lhs = 0
            for j in range(n_dec):
                ine_lhs += ine_mat[i, j] * dec_vec[j]
            m.addConstr(ine_lhs <= ine_rhs)

    # OPTIMIZE
    m.optimize()

    # PRINT RESULTS
    for v in m.getVars():
        print('%s %g' % (v.varName, v.x))
    print('Obj: %g' % obj.getValue())

    # RETURN DECISON VARIABLES
    return np.array([v.x for v in m.getVars()])



class Trajectory():
    # REFRERNCES
    # EXPLANATION

    def __init__(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, t=None, rounds=False):
        # Initializes an instance with pose coordinates of keyframes {x, y, z, roll, pitch, yaw} 
        # and corresponding time coordinates of keyframes.
        # For calculating a closed trajectory with specific amount of rounds,
        # set the rounds= parameter equal to a positive integer.

        # Format set coordinates, so they are numpy arrays with shape (?,)
        coordinates = [x, y, z, roll, pitch, yaw, t]
        for i in range(len(coordinates)):
            if coordinates[i] is not None:
                try:
                    coordinates[i][:, 0]
                    print('ERROR (class Trajectory, def __init__):\n\t All coordinate arrays {x, y, z, roll, pitch, yaw, t} must have only one dimension.')
                    return
                except:
                    coordinates[i] = np.squeeze(np.array(coordinates[i]))

        # Check if set coordinates\time  have the same amount of elements
        set_coordinates = [coordinate for coordinate in coordinates if coordinate is not None]
        n = set_coordinates[0].shape[0]
        for set_coordinate in set_coordinates[:-1]:
            if set_coordinate.shape[0] != n:
                print('ERROR (class Trajectory, def __init__):\n\t All coordinate arrays {x, y, z, roll, pitch, yaw, t} must have the same amount of elements.')
                return

        # Check if there are at least 2 keyframes
        if n < 2:
            print('ERROR (class Trajectory, def __init__):\n\t Number of keyframes n must be >= 2.')
            return
        
        if rounds != False:
            if type(rounds) != int or rounds < 1:
                print('ERROR (class Trajectory, def __init__): rounds must be semi-positive integer or False.')
                return
            # Check if there is there is an additional time element
            if set_coordinates[-1].shape[0] != n + 1:
                print('ERROR (class Trajectory, def __init__):\n\t If rounds is specified, the time array should have one more element than the other coordinate arrays.')
                return
            else:
                for i in range(6):
                    coordinates[i] = np.tile(coordinates[i], rounds)
                    coordinates[i] = np.append(coordinates[i], coordinates[i][0])
                t_end = coordinates[-1][-1]
                coordinates[-1] = np.tile(coordinates[-1][:-1], rounds)
                coordinates[-1] = np.append(coordinates[-1], t_end)
                for i in range(1, rounds):
                    coordinates[-1][i * n : (i + 1) * n] += i * coordinates[-1][-1]
                coordinates[-1][-1] += (rounds - 1) * coordinates[-1][-1]
                n = rounds * n + 1
        else:
            # Check if set coordinates have the same amount of elements
            if set_coordinates[-1].shape[0] != n:
                print('ERROR (class Trajectory, def __init__):\n\t All coordinate arrays {x, y, z, roll, pitch, yaw, t} must have the same amount of elements.')
                return

        # Initialize 
        self.poses = {
            'x'         :   coordinates[0],
            'y'         :   coordinates[1],
            'z'         :   coordinates[2],
            'roll'      :   coordinates[3],
            'pitch'     :   coordinates[4],
            'yaw'       :   coordinates[5],
            't'         :   coordinates[6],
            'n'         :   n
        }

        self.traj = {
            'n'         :   n - 1
        }

    @staticmethod
    def Debug():

        # *** INPUTS  ***
        x = np.array([      0,      1.5,        3,      2,      0,      -3,     -2,     0,          ], dtype=np.float64)
        y = np.array([      0,       1.5,        3,      4,      6,       3,     -5,     -4,         ], dtype=np.float64)
        z = np.array([      0,      .1,         0.2,    0.2,    0.15,   0.13,   0.1,    0.07,       ], dtype=np.float64)
        yaw = np.array([      0,      45,         90,     135,    180,    270,    290,    340,          ], dtype=np.float64)
        t = np.array([      0,      1.0,    2,          3,      4,      5,      6,      7,      8       ] , dtype=np.float64)
        # *** \INPUTS  ***

        min_snap_traj = Trajectory( x=x, y=y, z=z, yaw=yaw, t=t, rounds=2)
        min_snap_traj.MinSnapTraj(6, solver='gurobi')    


    def DerivMat(self, ord_poly, ord_deriv):

        if type(ord_poly) != int or ord_poly < 0  or type(ord_deriv) != int or ord_deriv < 0:
            print('ERROR (class Trajectory, def DerivMat): ord_poly and ord_deriv must be semi-positive integers.')
            return
        
        # First order derivative Matrix 
        deriv_mat = np.zeros( (ord_poly, ord_poly) )
        for i in range(1, ord_poly):
            deriv_mat[i - 1, i] = i
        
        # ${ord_deriv}-th order derivative matrix
        deriv_mat = np.linalg.matrix_power( deriv_mat, ord_deriv )

        return deriv_mat

    
    def QP_ObjMat(self, ord_poly, ord_deriv, mu):
        # QP: matrix in objective function
        # (n_var x n_var) - dimensional real symmetric matrix Q

        deriv_mat = self.DerivMat(ord_poly, ord_deriv)

        # For every single trajectory between two keyframes it is the same
        QP_obj_mat_blocks = [ np.matmul(deriv_mat.T, deriv_mat) ] * self.traj['n']

        # Build block diagonal matrix for whole trajectory
        obj_mat = mu * scipy.linalg.block_diag(*QP_obj_mat_blocks)

        return obj_mat
    

    def QP_Equ_Deriv0(self, ord_poly, QP):

        # Amount of all coefficients of all trajectory segments
        n_coeffs = ord_poly * self.traj['n']

        # Polynomial powered t array
        t_poly = self.poses['t'][:, np.newaxis] ** np.arange(ord_poly)[np.newaxis, :]

        # Allocation of matrix and vector in equality constraints
        equ_matt1 = np.zeros( (self.traj['n'], n_coeffs) )
        equ_matt2 = np.zeros( (self.traj['n'], n_coeffs) )
        equ_vec_t1 = np.zeros( (self.traj['n'], ) )
        equ_vec_t2 = np.zeros( (self.traj['n'], ) )

        # Make sure that for every trajectory segment,
        # when they begin and end,
        # they reached the coordinates of the keyframe of that specific time point.
        for i in range( self.traj['n'] ):

            # Write polynomial time into the matrix            
            equ_matt1[i, i * ord_poly : (i + 1) * ord_poly] = t_poly[i, :][np.newaxis, :]
            equ_matt2[i, i * ord_poly : (i + 1) * ord_poly] = t_poly[i + 1, :][np.newaxis, :]

            # Write space coordinate into vec
            equ_vec_t1[i] = self.poses[QP][i]
            equ_vec_t2[i] = self.poses[QP][i + 1]

        equ_matderiv0 = np.vstack( 
            (
                equ_matt1, 
                equ_matt2
            ) 
        )
        equ_vec_deriv0 = np.vstack(
            (
                equ_vec_t1[:, np.newaxis], 
                equ_vec_t2[:, np.newaxis]
            )
        )
        
        return [equ_matderiv0, equ_vec_deriv0]


    def QP_Equ_Deriv_(self, ord_poly, ord_deriv, QP):

        # Amount of all coefficients of all trajectory segments
        n_coeffs = ord_poly * self.traj['n']

        # Polynomial powered t array
        t_poly = self.poses['t'][:, np.newaxis] ** np.arange(ord_poly)[np.newaxis, :]

        # Spline (trajectory) should be ${cont_dif} times continuously differentiable
        #OLD ord_dif = ord_poly - 2
        ord_dif = ord_deriv - 2  # MINIMUM VALUE ??? TRY BIGGER VALUES

        # Per derivative, there is one condition for each inner traj
        equ_matderiv_ = np.zeros( ((self.traj['n'] - 1) * ord_dif, n_coeffs) )
        equ_vec_deriv_ = np.zeros( ((self.traj['n'] - 1) * ord_dif, ) )

        # For each derivative which should be smooth
        for j in range(0, ord_dif):

            # Compute corresponding derivative matrix
            deriv_mat = self.DerivMat(ord_poly, j + 1)

            # For each inner keyframe, each trajectory
            for i in range( self.traj['n'] - 1 ):

                row = j * (self.traj['n'] - 1) + i
                cols1 = range( ord_poly * i, ord_poly * (i + 1) )
                cols2 = range( ord_poly * (i + 1), ord_poly * (i + 2) )

                equ_matderiv_[row, cols1] = np.matmul(t_poly[i + 1], deriv_mat)
                equ_matderiv_[row, cols2] = -np.matmul(t_poly[i + 1], deriv_mat)

        # KITTING it ROUND WISE APPROACH OUTER KEYFRAMES
        equ_matderiv_outer_ = np.zeros( (ord_dif, n_coeffs) )
        equ_vec_deriv__outer = np.zeros( (ord_dif, ) )
            
        for j in range(0, ord_dif):

            deriv_mat = self.DerivMat(ord_poly, j + 1)
            
            row = j
            cols1 = range( 0, ord_poly )
            cols2 = range( ord_poly * (self.traj['n'] - 1), ord_poly * self.traj['n'] )

            equ_matderiv_outer_[row, cols1] = np.matmul(t_poly[ 0 ], deriv_mat)
            equ_matderiv_outer_[row, cols2] = -np.matmul(t_poly[ - 1 ], deriv_mat)

        equ_matderiv_ = np.vstack( (equ_matderiv_, equ_matderiv_outer_) )
        equ_vec_deriv_ = np.squeeze( 
            np.vstack( 
                (equ_vec_deriv_[:, np.newaxis], equ_vec_deriv__outer[:, np.newaxis])
            )
        )

        return [equ_matderiv_, equ_vec_deriv_]
            

    def QP_Equ(self, ord_poly, ord_deriv, QP):
        # This function builds the matrix and the vector
        # which appear in the equality constrains.

        #n = self.poses['n']
        #n = self.traj['n']
        #t = self.poses['t']
        #coords = self.poses[coord]

        # Calculate the equality constrains related to the 0th derivative of the polynom.
        # Ensures that polynoms end and start at their corresponding keyframe
        # -> 2 equations per polynom
        equ_matderiv0, equ_vec_deriv0 = self.QP_Equ_Deriv0(ord_poly, QP)

        # Calculate the equality constrains related to the derivatives of order 1 to (ord_poly - 2) of the polynom.
        # Ensures that polynoms are (ord_poly - 2) times continuously differentiable at the keyframes.
        # -> (ord_poly - 2) equations per polynom
        equ_matderiv_, equ_vec_deriv_ = self.QP_Equ_Deriv_(ord_poly, ord_deriv, QP)

        # Combine all constrains
        equ_mat = np.vstack(
            (
                equ_matderiv0, 
                equ_matderiv_
            )
        )
        equ_vec = np.vstack(
            (
                equ_vec_deriv0,
                equ_vec_deriv_[:, np.newaxis],
            )
        )

        return [equ_mat, equ_vec]
        

    def TestQPModel(self, obj_mat, equ_mat, QP):

        n_equ = equ_mat.shape[0]
        n_dec = equ_mat.shape[1]

        rank_equ_mat = np.linalg.matrix_rank( equ_mat )
        rank_obj_equ_mat = np.linalg.matrix_rank(np.vstack( (obj_mat, equ_mat) ))
        
        if not rank_equ_mat == n_equ:
            print '\n[WARNING]: For the QP of coordinate', QP, '\n\trank( equ_mat ) = ', rank_equ_mat, '\n\tnumber of equations = ', n_equ
        if not rank_obj_equ_mat == n_dec:
            print '\n[WARNING]: for the QP of coordinate', QP, '\n\trank( [obj_mat, equ_mat]^T ) = ', rank_obj_equ_mat, '\n\tnumber of decision variables = ', n_dec


    def QP_SolveCVXOPT(self, ord_poly, obj_mat, obj_vec=None, ine_mat=None, ine_vec=None, equ_mat=None, equ_vec=None):

        if obj_vec != None:
            obj_vec = cvxopt.matrix( obj_vec, tc='d' )
        else:
            obj_vec = cvxopt.matrix( np.zeros( (obj_mat.shape[0], ) ), tc='d' )

        obj_mat = cvxopt.matrix( obj_mat, tc='d' )

        constrains = [ine_mat, ine_vec, equ_mat, equ_vec]
        for i in range(len(constrains)):
            if constrains[i] is not None:
                constrains[i] = cvxopt.matrix( constrains[i], tc='d' )

        ine_mat = constrains[0]
        ine_vec = constrains[1]
        equ_mat = constrains[2]
        equ_vec = constrains[3]

        sol = cvxopt.solvers.qp(
            obj_mat, 
            obj_vec, 
            G=ine_mat, 
            h=ine_vec, 
            A=equ_mat, 
            b=equ_vec
        )

        poly_coeffs = np.array(sol['x'])

        poly_coeffs = np.reshape(
            poly_coeffs, 
            ( self.traj['n'], ord_poly )
        ).T

        return poly_coeffs


    def QP_SolveGurobi(self, ord_poly, obj_mat, obj_vec=None, ine_mat=None, ine_vec=None, equ_mat=None, equ_vec=None):
        
        poly_coeffs = Gurobi_SolveQPMatrixNotation(
            obj_mat,
            obj_vec=obj_vec,
            ine_mat=ine_mat,
            ine_vec=ine_vec,
            equ_mat=equ_mat,
            equ_vec=equ_vec,
            sense=gurobipy.GRB.MINIMIZE,
            lb=-gurobipy.GRB.INFINITY, 
            ub=gurobipy.GRB.INFINITY, 
            vtype=gurobipy.GRB.CONTINUOUS
        )

        poly_coeffs = np.reshape(
            poly_coeffs, 
            ( self.traj['n'], ord_poly )
        ).T

        return poly_coeffs


    def QP_Solve(self, ord_poly, obj_mat, obj_vec=None, ine_mat=None, ine_vec=None, equ_mat=None, equ_vec=None, solver='gurobi'):

        if solver == 'gurobi':

            return self.QP_SolveGurobi(ord_poly, obj_mat, obj_vec=obj_vec, ine_mat=ine_mat, ine_vec=ine_vec, equ_mat=equ_mat, equ_vec=equ_vec)

        if solver == 'cvxopt':

            return self.QP_SolveCVXOPT(ord_poly, obj_mat, obj_vec=obj_vec, ine_mat=ine_mat, ine_vec=ine_vec, equ_mat=equ_mat, equ_vec=equ_vec)


    def PolyToTraj(self, ord_poly, poly_coeffs, dt=1e-3):
        # time step, dt

        # Number of sample points for traj based on dt
        n_samp =  int( (self.poses['t'][-1] - self.poses['t'][0]) / dt )

        # The time axis of the traj
        t_traj = np.linspace(
            self.poses['t'][0], 
            self.poses['t'][-1], 
            num=n_samp,
            endpoint=False
        )

        # The polynomial time axis of the traj
        t_poly = t_traj[ :, np.newaxis ] ** np.arange( ord_poly )[ np.newaxis, : ]

        # Allocate the coordinates of the traj
        coord_traj = np.zeros( t_traj.shape, dtype=np.float64 )

        # Calculate each segment of the traj with the right poly coeffs
        for seg in range(self.traj['n']):

            # Indices of the current segment
            ind_t_seg = np.where( (t_traj >= self.poses['t'][seg]) & (t_traj < self.poses['t'][seg + 1]) )[0]

            # Compute coordinates of the current segment and insert into global traj
            coord_traj[ind_t_seg] = np.matmul(t_poly[ind_t_seg, :], poly_coeffs[:, seg])

        return t_traj, coord_traj


    def PlotTraj(self, traj_method, plane=True):

        try:
            self.traj[traj_method]
        except:
            print('\n [ERROR] Before plotting, compute the trajectory with the method: ' + str(traj_method))
            return
        
        traj_t      =   np.squeeze( self.traj[traj_method]['t'] )
        traj_x      =   np.squeeze( self.traj[traj_method]['x'] )
        traj_y      =   np.squeeze( self.traj[traj_method]['y'] )
        traj_z      =   np.squeeze( self.traj[traj_method]['z'] )

        xmax        =   np.max( traj_x )
        ymax        =   np.max( traj_y )
        zmax        =   np.max( traj_z )

        xmin        =   np.min( traj_x )
        ymin        =   np.min( traj_y )
        zmin        =   np.min( traj_z )

        keyframe_x  =   np.squeeze( self.poses['x'] )
        keyframe_y  =   np.squeeze( self.poses['y'] )
        keyframe_z  =   np.squeeze( self.poses['z'] )


        mpl.rcParams['legend.fontsize'] = 10
        fig = plt.figure('PlotTraj')
        ax = fig.gca(projection='3d') 
        ax.plot(traj_x, traj_y, traj_z, label='Trajectory')
        ax.plot(traj_x[traj_z < 0], traj_y[traj_z < 0], traj_z[traj_z < 0], label='Underground', color='r')
        ax.scatter(keyframe_x, keyframe_y, keyframe_z, s=200, label='Keyframes')

        if plane:

            plane_point  = np.array( [0, 0, 0] )
            plane_normal = np.array( [0, 0, 1] )
            # a plane is a*x+b*y+c*z+d=0
            # [a,b,c] is the normal. Thus, we have to calculate
            # d and we're set
            d = -plane_point.dot(plane_normal)
            xx, yy = np.meshgrid(range(-10, 10), range(-10, 10))
            z = (-plane_normal[0] * xx - plane_normal[1] * yy - d) * 1. / plane_normal[2]

            ax.plot_surface(xx, yy, z, alpha=0.2, color='g')

        ax.legend()

        xmar = (xmax - xmin) / 10
        ymar = (ymax - ymin) / 10
        zmar = (zmax - zmin) / 10
        ax.set_xlim(xmin - xmar, xmax + xmar)
        ax.set_ylim(ymin - ymar, ymax + ymar)
        ax.set_zlim(zmin - zmar, zmax + zmar)
        
        plt.show()


    def PlotTrajAnim(self, traj_method, plane=True):

        # Check if trajectory is already computed
        try:
            self.traj[traj_method]
        except:
            print('\n [ERROR] Before plotting, compute the trajectory with the method: ' + str(traj_method))
            return
        
        # *** DATA ***
        t      =   np.squeeze( self.traj[traj_method]['t'] )
        x      =   np.squeeze( self.traj[traj_method]['x'] )
        y      =   np.squeeze( self.traj[traj_method]['y'] )
        z      =   np.squeeze( self.traj[traj_method]['z'] )

        xmax        =   np.max( x )
        ymax        =   np.max( y )
        zmax        =   np.max( z )

        xmin        =   np.min( x )
        ymin        =   np.min( y )
        zmin        =   np.min( z )

        xmar        =   (xmax - xmin) / 10
        ymar        =   (ymax - ymin) / 10
        zmar        =   (zmax - zmin) / 10

        xlim        =   [xmin - xmar, xmax + xmar]
        ylim        =   [ymin - ymar, ymax + ymar]
        zlim        =   [zmin - zmar, zmax + zmar]

        data = np.vstack( (t[np.newaxis, :], x[np.newaxis, :], y[np.newaxis, :], z[np.newaxis, :]) )

        keyframe_x  =   np.squeeze( self.poses['x'] )
        keyframe_y  =   np.squeeze( self.poses['y'] )
        keyframe_z  =   np.squeeze( self.poses['z'] )

        ival = 50e-3
        fra = int((t[-1] - t[0]) / ival)
        data = data[:, 0::int(data.shape[1] / fra)]


        # *** ANIMATION ***
        # initialization function: plot the background of each frame
        def init(x, y, z, keyframe_x, keyframe_y, keyframe_z):

            ax.plot(x, y, z, label='Trajectory')
            ax.plot(x[z < 0], y[z < 0], z[z < 0], label='Collison', color='r')
            ax.scatter(keyframe_x, keyframe_y, keyframe_z, s=500, label='Keyframes')

            if plane:

                plane_point  = np.array( [0, 0, 0] )
                plane_normal = np.array( [0, 0, 1] )
                # a plane is a*x+b*y+c*z+d=0
                # [a,b,c] is the normal. Thus, we have to calculate
                # d and we're set
                d = -plane_point.dot(plane_normal)
                xx, yy = np.meshgrid(range(-10, 10), range(-10, 10))
                z = (-plane_normal[0] * xx - plane_normal[1] * yy - d) * 1. / plane_normal[2]

                ax.plot_surface(xx, yy, z, alpha=0.2, color='g', label='Ground Plane')

            drone_pos.set_data( data[1 : 3, 0:2] )

            
            #ax.legend()

            return drone_pos

        def update_position(num, data, drone_pos):
            drone_pos.set_data( data[1 : 3, num:num+2] )
            drone_pos.set_3d_properties( data[3, num:num+2] )
            drone_pos.set_label( 'Drone Position, t=' + str(np.floor(data[ 0, num ])) + 's' )
            ax.legend()
            return drone_pos

        # Attaching 3D axis to the figure
        # mpl.rcParams['legend.fontsize'] = 10
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        #ax.legend()


        # ax.plot() returns a tuple with one element. By adding the comma to the assignment target list, you ask Python to unpack the return value and assign it to each variable named to the left in turn.
        drone_pos = ax.plot(
            data[1, 0:1],
            data[2, 0:1],
            data[3, 0:1],
            lw=15, 
            label='Drone Position, t=' + str(t[ 0 ]) + 's',
            color='y'
        )[0]

        # Setting the axes properties
        ax.set_xlim3d(xlim)
        ax.set_xlabel('X')
        ax.set_ylim3d(ylim)
        ax.set_ylabel('Y')
        ax.set_zlim3d(zlim)
        ax.set_zlabel('Z')
        ax.set_title(traj_method + ' TRAJECTORY')

        init(x, y, z, keyframe_x, keyframe_y, keyframe_z)

        # call the animator.  blit=True means only re-draw the parts that have changed.
        position_ani = animation.FuncAnimation(
            fig, 
            update_position, 
            #init_func=init(x, y, z, keyframe_x, keyframe_y, keyframe_z),
            frames=fra,
            fargs=(data, drone_pos), 
            interval=ival*1e3, 
            blit=False)

        # save the animation as an mp4.  This requires ffmpeg or mencoder to be
        # installed.  The extra_args ensure that the x264 codec is used, so that
        # the video can be embedded in html5.  You may need to adjust this for
        # your system: for more information, see
        # http://matplotlib.sourceforge.net/api/animation_api.html
        #anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
        
        plt.show()
        
    
    def MinSnapTraj(self, ord_poly, solver='gurobi'):
        # Ressource ?????
        # Explanation ?????????

        self.traj[ 'min_snap' ] = {}
        
        # Order of the polynomial basis functions for trajectory
        # Here: ord_poly = 3 -> 1 + x + x^2

        # *** PARAMETERS ***
        # ------------------
        # Order of derivative of position squared, to be minimized
        ord_deriv_pos = 4 # Snap
        # Order of derivative of yaw angle squared, to be minimized
        ord_deriv_ang = 2
        # Constant that makes the position-integrand nondimensional
        mu_pos = 1
        # constant that makes the yaw-angle-integrand nondimensional
        mu_ang = 1
        # -------------------

        # *** DEVELOP QP MODEL ***
        # ------------------------
        # Define 4 independent QP models for x, y, z, yaw
        QPs =	['x', 'y', 'z', 'yaw']
        ord_derivs = {
            "x"     :   ord_deriv_pos,
            "y"     :   ord_deriv_pos,
            "z"     :   ord_deriv_pos,
            "yaw"   :   ord_deriv_ang
        }
        ord_polys = {
            "x"     :   ord_poly,
            "y"     :   ord_poly,
            "z"     :   ord_poly,
            "yaw"   :   ord_poly
        }
        mus = {
            "x"     :   mu_pos,
            "y"     :   mu_pos,
            "z"     :   mu_pos,
            "yaw"   :   mu_ang
        }
        #obj_mats = {}
        #equ_mats = {}
        #polys_coeffs = {}
        


        
        for QP in QPs:

            ord_deriv = ord_derivs[ QP ]
            ord_poly = ord_polys[ QP ]
            mu = mus[ QP ]
            
            # Calculate the matrix appearing in the objective function of the quadratic program
            obj_mat = self.QP_ObjMat( ord_poly, ord_deriv, mu )
            #obj_mats[ QP ] = self.QP_ObjMat( ord_poly, ord_deriv, mu ) # MAYBE NOT NEEDED!!!

            ########## UNTIL HERE CORRECT

            equ_mat, equ_vec = self.QP_Equ( ord_poly, ord_deriv, QP )
            # Format of QPs: [[ord_poly, ord_deriv, mu, coordinates], obj_mat, [equ_mat, equ_vec]]
            #equ_mats[ QP ] = self.QP_Equ( ord_poly, ord_deriv, QP ) # MAYBE NOT NEEDED!!!

            # Test model if its rank has defiency
            self.TestQPModel( obj_mat, equ_mat, QP )

            # Solve Model get polynom coefficients for traj segments
            poly_coeffs = self.QP_Solve(
                ord_poly,
                obj_mat, 
                equ_mat=equ_mat, 
                equ_vec=equ_vec,
                solver=solver
            )
            # Calculate trajectory from polynomial coefficients
            self.traj['min_snap']['t'], self.traj['min_snap'][QP] = self.PolyToTraj( ord_poly, poly_coeffs, dt=1e-3 )

        # Print trajectory
        #self.PlotTraj('min_snap', plane=True)
        self.PlotTrajAnim('min_snap', plane=True)

            
            




if __name__ == "__main__":


    
    Trajectory.Debug()

    
    


    #####################



    