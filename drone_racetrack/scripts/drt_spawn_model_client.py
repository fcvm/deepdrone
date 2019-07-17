#!/usr/bin/env python
# -*- coding: utf-8 -*-



'____ TO DO ____'

# - ADDITIONAL GATE MODELS [ ]

'____ \TO DO ____'



'____ IMPORT MODULES ____'

import os; dirname = os.path.dirname(__file__)
import sys
import numpy as np
from scipy.spatial.transform import Rotation

# ROS
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

# Own scripts
from drt_creator_frame import *

'____ \IMPORT MODULES ____'



'____ DRONE RACETRACK CLASS ____'

class DRT_Frame( DRT_CreatorFrame ):

    def __init__( self, master ):

        # Initialize first keyframe at origin
        DRT_CreatorFrame.__init__( self, master )

        # Spawn the first keyframe at the origin
        self.SpawnGateRequest()


    def BuildClick( self, name ):
        # The following happens when the build button is pressed.

        # If the build button is raised
        if self.sab_frames[ name ][ 'enabled' ] == True:

            # Add the next keyframe and spawn in Gazebo
            self.AddKeyframe()
            self.SpawnGateRequest()

            # Now that there is at least one additional keyframe, the wreck button is raised.
            name = 'wreck'
            self.sab_frames[ name ][ 'enabled' ] = True
            self.sab_frames[ name ][ 'but' ].config(
                relief=tk.RAISED,
                image=self.sab_frames[ name ][ 'icons' ][ self.sab_frames[ name ][ 'enabled' ] ]
            )

            # Now that there is at least one additional keyframe, the trajectory button is raised.
            name = 'traj'
            self.sab_frames[ name ][ 'enabled' ] = True
            self.sab_frames[ name ][ 'but' ].config(
                relief=tk.RAISED,
                image=self.sab_frames[ name ][ 'icons' ][ self.sab_frames[ name ][ 'enabled' ] ]
            )

            # Print keyframes on terminal
            print("\nGate has been built.\n\nKeyframes:\n")
            print '\tx:\t', self.keyframes[ 'x' ]
            print '\ty:\t', self.keyframes[ 'y' ]
            print '\tz:\t', self.keyframes[ 'z' ]
            print '\troll:\t', self.keyframes[ 'roll' ]
            print '\tpitch:\t', self.keyframes[ 'pitch' ]
            print '\tyaw:\t', self.keyframes[ 'yaw' ]
            print '\tt:\t', self.keyframes[ 't' ]
            print '\tn:\t', self.keyframes[ 'n' ]
            
        else:
            print("\nCannot build gate.")


    def SpawnGateRequest( self ):

        # Model name in Gazebo: Gate***
        model_name = 'Gate' + format( self.keyframes[ 'n' ] - 1, '03' )
        # Read in the SDF file
        f = open(os.path.abspath(os.path.join(dirname,'../models/gate/model.sdf')),'r')
        model_xml = f.read()
        # Namespace ??
        robot_namespace = 'racetrack'
        # Reference frame ??
        reference_frame = 'world'
        
        # Load the current keyframe coordinates
        x = self.keyframes[ 'x' ][ -1 ]
        y = self.keyframes[ 'y' ][ -1 ]
        z = self.keyframes[ 'z' ][ -1 ]
        roll = self.keyframes[ 'roll' ][ -1 ]
        pitch = self.keyframes[ 'pitch' ][ -1 ]
        yaw = self.keyframes[ 'yaw' ][ -1 ]
        # Transfer Euler angles to quaternion
        quat = Rotation.from_euler( 'xyz', [roll, pitch, yaw] ).as_quat()
        # Message classes Pose, Point, Quaternion of geometry_msgs.msg
        current_keyframe = Pose( Point( x, y, z ), Quaternion ( quat[ 0 ], quat[ 1 ], quat[ 2 ], quat[ 3 ] ) )
        
        # Service class SpawnModelRequest of gazebo_msgs.srv
        spawn_model_request = SpawnModelRequest(
            model_name, 
            model_xml, 
            robot_namespace, 
            current_keyframe, 
            reference_frame
        )
        self.SpawnModelClient( spawn_model_request )


    def SpawnModelClient( self, request ):

        # Wait that gazebo_ros is running
        rospy.wait_for_service('gazebo/spawn_sdf_model')

        try:
            spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            response = spawn_sdf_model(request)
            print '\nGate spawned: ', response.success
        except rospy.ServiceException, e:
            print "Spawn call failed: %s"%e


    def WreckClick( self, name ):
        # The following happens when the wreck button is pressed:

        # If the wreck button is raised
        if self.sab_frames[ name ][ 'enabled' ] == True:
            
            # Delete the current keyframe and delete corresponding model in Gazebo
            self.DeleteGateRequest()
            self.DelKeyframe()

    
            # After wrecking the current gate, if there is only the gate in the
            # origin left, disable the delete button and the trajectory button.
            if self.keyframes[ 'n' ] == 1:

                name = 'wreck'
                self.sab_frames[ name ][ 'enabled' ] = False
                self.sab_frames[ name ][ 'but' ].config(
                    relief=tk.SUNKEN,
                    image=self.sab_frames[ name ][ 'icons' ][ self.sab_frames[ name ][ 'enabled' ] ]
                )

                name = 'traj'
                self.sab_frames[ name ][ 'enabled' ] = False
                self.sab_frames[ name ][ 'but' ].config(
                    relief=tk.SUNKEN,
                    image=self.sab_frames[ name ][ 'icons' ][ self.sab_frames[ name ][ 'enabled' ] ]
                )

            # Print keyframes on terminal
            print("\nGate has been wrecked.\n\nKeyframes:\n")
            print '\tx:\t', self.keyframes[ 'x' ]
            print '\ty:\t', self.keyframes[ 'y' ]
            print '\tz:\t', self.keyframes[ 'z' ]
            print '\troll:\t', self.keyframes[ 'roll' ]
            print '\tpitch:\t', self.keyframes[ 'pitch' ]
            print '\tyaw:\t', self.keyframes[ 'yaw' ]
            print '\tt:\t', self.keyframes[ 't' ]
            print '\tn:\t', self.keyframes[ 'n' ]

        else:
            print("\nInitial gate cannot be wrecked.")


    def DeleteGateRequest( self ):

        # Name of current gate
        model_name = 'Gate' + format( self.keyframes[ 'n' ] - 1, '03' )
        # Service class DeleteModelRequest of gazebo_msgs.srv
        delete_model_request = DeleteModelRequest( model_name )
        self.DeleteModelClient( delete_model_request )


    def DeleteModelClient( self, request ):

        # Wait that gazebo_ros is running
        rospy.wait_for_service( 'gazebo/delete_model' )

        try:
            delete_model = rospy.ServiceProxy( 'gazebo/delete_model', DeleteModel )
            response = delete_model(request)
            print '\nGate deleted: ', response.success
        except rospy.ServiceException, e:
            print "Delete call failed: %s"%e

'____ \DRONE RACETRACK CLASS ____'



'____ MAIN ____'

if __name__ == "__main__":

    rospy.init_node('SpawnModelClient', log_level=rospy.INFO)

    # *** WINDOW *** #
    root = tk.Tk()
    # *** FRAME *** #
    DroneRacetrackNode = DRT_Frame(root)
    # Keep window continuosly open until close button pressed
    root.mainloop()

'____ \MAIN ____'