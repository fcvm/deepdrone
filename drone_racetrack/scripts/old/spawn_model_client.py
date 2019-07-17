#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import rospy

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

from scipy.spatial.transform import Rotation

# Own scripts
from DroneRaceTrackTycoon import *
#from trajectory_generation import *








class DRT_Frame(DRT_CreatorFrame):

    def __init__(self, Master):

        # Initialize first keyframe at origin
        self.keyframes = {
            'x'         :   [ 0 ],
            'y'         :   [ 0 ],
            'z'         :   [ 0 ],
            'roll'      :   [ 0 ],
            'pitch'     :   [ 0 ],
            'yaw'       :   [ 0 ],
            't'         :   [ 0 ],
            'n'         :     1
        }

        # Relative coordinates to the current keyframe when building a new keyframe         # SET THIS VALUES THROUGH GUI !!!!!
        self.gate_diameter = 1
        self.keyframe_delta = {
            'x'         :   np.array( [   1.50,    3.00,    4.50,    3.00,    4.50,    3.00,    1.50 ] ) * self.gate_diameter,
            'y'         :   np.array( [   0.50,    1.00,    1.50,    0.00,   -1.50,   -1.00,   -0.50 ] ) * self.gate_diameter,
            'z'         :   np.array( [  -1.00,    0.00,    1.00 ] ),
            'roll'      :   np.array( [   0.00 ] ) * self.gate_diameter,
            'pitch'     :   np.array( [   0.00 ] ),
            'yaw'       :   np.array( [   1/6,      1/6,     1/6,    0.00,    -1/6,    -1/6,    -1/6 ] ) * np.pi,
            't'         :   np.array( [   1.00 ] ),
        }

        # Spawn the first keyframe at the origin
        self.SpawnGateRequest()


    def BuildClick(self):
        # The following happens when the build button is pressed:

        # If the build button is raised
        if self.buildEnabled == True:

            # Add the next keyframe and spawn it in Gazebo
            self.AddKeyframe()
            self.SpawnGateRequest()

            '''
            # Check if the racetrack is closed
            if (self.Poses[0] == self.Poses[-1]).all():
                self.buildEnabled = False
                self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.SUNKEN)
            '''

            # Now that there is at least one additional keyframe, the wreck button is raised.
            self.wreckEnabled = True
            self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.RAISED)

            # Now that there is at least one additional keyframe, the trajectory button is raised.
            self.trajEnabled = True
            self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.RAISED)
        
        '''
        # If building is not possible, it means the racetrack is already closed
        else:
            print("Race track is already closed.")
        '''


    def AddKeyframe(self):

        # The coordinates of the current keyframe
        x_current = self.keyframes[ 'x' ][ -1 ]
        y_current = self.keyframes[ 'y' ][ -1 ]
        z_current = self.keyframes[ 'z' ][ -1 ]
        roll_current = self.keyframes[ 'roll' ][ -1 ]
        pitch_current = self.keyframes[ 'pitch' ][ -1 ]
        yaw_current = self.keyframes[ 'yaw' ][ -1 ]
        t_current = self.keyframes[ 't' ][ -1 ]
        n = self.keyframes[ 'n' ]

        # The relative coordinates of the next keyframe
        x_delta = self.keyframe_delta[ 'x' ][ self.directionSelection[0] ]
        y_delta = self.keyframe_delta[ 'y' ][ self.directionSelection[0] ]
        z_delta = self.keyframe_delta[ 'z' ][ self.slopeSelection[0] ]
        roll_delta = self.keyframe_delta[ 'roll' ][ 0 ]
        pitch_delta = self.keyframe_delta[ 'pitch' ][ 0 ]
        yaw_delta = self.keyframe_delta[ 'yaw' ][ self.directionSelection[0] ]
        t_delta = self.keyframe_delta[ 't' ][ 0 ]

        # Rotate the relative coordinates (x, y, z) of the next keyframe
        # according to the orientation of the current keyframe.
        rot_mat = Rotation.from_euler('xyz', [roll_current, pitch_current, yaw_current]).as_dcm()
        pos_delta = np.matmul( rot_mat, np.array( [x_delta, y_delta, z_delta] )[ :, np.newaxis ] )

        # The absolute coordinates of the next keyframe
        self.keyframes[ 'x' ].append( x_current + pos_delta[ 0 ] )
        self.keyframes[ 'y' ].append( y_current + pos_delta[ 1 ] )
        self.keyframes[ 'z' ].append( z_current + pos_delta[ 2 ] )
        self.keyframes[ 'roll' ].append( roll_current + roll_delta )
        self.keyframes[ 'pitch' ].append( pitch_current + pitch_delta )
        self.keyframes[ 'yaw' ].append( yaw_current + yaw_delta )
        self.keyframes[ 't' ].append( t_current + t_delta )
        self.keyframes[ 'n' ] = n + 1
        
        '''
        # Rotate the relative coordinates (x, y, z) of the second next keyframe
        # according to the orientation of the next keyframe.
        rot_mat = Rotation.from_euler('xyz', [keyframe_roll, keyframe_pitch, keyframe_yaw]).as_dcm()
        '''


    def WreckClick(self):
        # The following happens when the wreck button is pressed:

        # If the wreck button is raised
        if self.wreckEnabled == True:

            # Delete the current gate model in Gazebo
            self.DeleteGateRequest()
            
            # Delete the coordinates of the current gate
            for coord in [ 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 't' ]:
                del self.keyframes[ coord ][ -1 ]
            self.keyframes[ 'n' ] -= 1
            
            ''' 
            backRotation = - self.Turns[-1]
            del self.Turns[-1]
            

            rot_mat = np.array([[np.cos(backRotation), -np.sin(backRotation)], 
                                   [np.sin(backRotation), np.cos(backRotation)]])
            '''
            
            # After wrecking the current gate, if there is only the gate in the
            # origin left, disable the delete button and the trajectory button.
            if self.keyframes[ 'n' ] == 1:

                self.wreckEnabled = False
                self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.SUNKEN)

                self.trajEnabled = False
                self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.SUNKEN)

            '''
            # After wrecking a gate, building another one is always possible
            self.buildEnabled = True
            self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.RAISED)
            '''
        else:
            print("Before wrecking a gate, build one first!")


    def SpawnGateRequest(self):

        # Model name in Gazebo: Gate***
        model_name = 'Gate' + format( self.keyframes[ 'n' ] - 1, '03' )
        # Read in the SDF file          # ADDITIONAL MODELS?
        dirname = os.path.dirname(__file__)
        f = open(os.path.abspath(os.path.join(dirname,'../models/gate/model.sdf')),'r')
        model_xml = f.read()
        # Namespace ??
        robot_namespace = 'racetrack'
        # Reference frame ??
        reference_frame = 'world'
        
        # Load the current keyframe position data
        x = self.keyframes[ 'x' ][ -1 ]
        y = self.keyframes[ 'y' ][ -1 ]
        z = self.keyframes[ 'z' ][ -1 ]
        # Load the current keyframe quaternion data
        xx, yy, zz, ww = self.CurrentQuaternion()
        # Message classes Pose, Point, Quaternion of geometry_msgs.msg
        current_keyframe = Pose( Point( x, y, z ), Quaternion ( xx, yy, zz, ww ) )
        
        # Service class SpawnModelRequest of gazebo_msgs.srv
        spawn_model_request = SpawnModelRequest(
            model_name, 
            model_xml, 
            robot_namespace, 
            current_keyframe, 
            reference_frame
        )
        self.SpawnModelClient(spawn_model_request)


    def CurrentQuaternion(self):

        roll = self.keyframes[ 'roll' ][ -1 ]
        pitch = self.keyframes[ 'pitch' ][ -1 ]
        yaw = self.keyframes[ 'yaw' ][ -1 ]

        # Rotation in Euler angles (intrinsic or extrinsic)???
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        # Rotation as quaternion
        quat = rotation.as_quat()

        # Return [xx, yy, zz, ww]
        return [quat[0], quat[1], quat[2], quat[3]]


    def DeleteGateRequest(self):

        # Name of current gate
        model_name = 'Gate' + format( self.keyframes[ 'n' ] - 1, '03' )
        # Service class DeleteModelRequest of gazebo_msgs.srv
        delete_model_request = DeleteModelRequest(model_name)
        self.DeleteModelClient(delete_model_request)


    def SpawnModelClient(self, request):

        # Wait that gazebo_ros is running
        rospy.wait_for_service('gazebo/spawn_sdf_model')

        try:
            spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            response = spawn_sdf_model(request)
            print '\nGate spawned: ', response.success
        except rospy.ServiceException, e:
            print "Spawn call failed: %s"%e


    def DeleteModelClient(self, request):

        # Wait that gazebo_ros is running
        rospy.wait_for_service('gazebo/delete_model')

        try:
            delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            response = delete_model(request)
            print '\nGate deleted: ', response.success
        except rospy.ServiceException, e:
            print "Delete call failed: %s"%e


    def trajClick(self):
        if self.trajEnabled == True:
            print("Click.")
            # self.Poses.append(np.array([[1, 0, 0], [1, 0, 0]]))
            #print((self.Poses[-1])[0, :])

            keyframes = np.zeros((len(self.Poses), 4))
            i = -1

            for pose in self.Poses:
                pose = np.array(pose)

                i += 1
                a = np.array([1, 0, 0])
                b = pose[:, 1]
                yaw = np.arccos(np.dot(a, b) / np.linalg.norm(b)) / np.pi * 180
                if b[1] < 0 :
                    yaw = 360 - yaw

                #print(pose)
                
                #print(pose[:, 0].T.shape, np.array([[yaw]]).shape)

                keyframe = np.hstack((pose[:, 0][np.newaxis, :], np.array([[yaw]])))
                keyframes[i, :] = keyframe
            
            print(len(self.Poses))

            t_keyframes = np.arange(len(self.Poses), dtype=np.float64)[:, np.newaxis] / (len(self.Poses) - 1)

            print(keyframes)

            print(t_keyframes)

            t, traj_x, traj_y, traj_z, traj_phi = get_snap_trajectory(keyframes, t_keyframes)

            # plot
            
            t = np.squeeze(t)
            traj_x = np.squeeze(traj_x)
            traj_y = np.squeeze(traj_y)
            traj_z = np.squeeze(traj_z)
            traj_phi = np.squeeze(traj_phi)

            mpl.rcParams['legend.fontsize'] = 10

            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.plot(traj_x, traj_y, traj_z, label='parametric curve')
            ax.scatter(keyframes[:, 0], keyframes[:, 1], keyframes[:, 2], s=200)
            ax.legend()
            plt.show()
            


        else:
            print("Button not enabled.")
        #if ((self.Poses[-1])[0, :] == np.array([0, 0, 0])).all():
        #    self.canBuild = False
        #    self.buildButton.config(image=self.unselectedImages, relief=tk.SUNKEN)



if __name__ == "__main__":

    rospy.init_node('SpawnModelClient', log_level=rospy.INFO)

    # *** WINDOW *** #
    root = tk.Tk()
    # *** FRAME *** #
    DroneRacetrackNode = DRT_Frame(root)
    # Keep window continuosly open until close button pressed
    root.mainloop()