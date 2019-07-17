#!/usr/bin/env python


import os
dirname = os.path.dirname(__file__)

import sys
import rospy

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion


from DroneRaceTrackTycoon import *
from trajectory_generation import *

from scipy.spatial.transform import Rotation as R






class DroneRacetrackNode(CreatorFrame):

    def __init__(self, Master):
        CreatorFrame.__init__(self, Master)
        self.spawn_gate_request()


    def buildClick(self):
        if self.buildEnabled == True:
            newPose = self.get_newPose(self.Poses[-1], self.directionSelection[0], self.slopeSelection[0])
            self.Poses.append(newPose)

            self.spawn_gate_request()
            print("Current Pose:\n" + str(self.Poses[-1]))

            if (self.Poses[0] == self.Poses[-1]).all():
                self.buildEnabled = False
                self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.SUNKEN)
            
            self.wreckEnabled = True
            self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.RAISED)

            self.trajEnabled = True
            self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.RAISED)
        else:
            print("Race track is already closed.")

    def get_newPose(self, currentPose, i_chosenDirection, i_chosenSlope):

        currentPosition = currentPose[:, 0][:, np.newaxis]
        currentOrientation = currentPose[:, 1][:, np.newaxis]

        nextPosition = np.zeros(currentPosition.shape)
        nextOrientation =  np.zeros(currentOrientation.shape)

        chosenDirection = self.directionValues[:, i_chosenDirection][:, np.newaxis]
        chosenOrientation = self.directionAngles[i_chosenDirection]

        self.Turns.append(chosenOrientation)

        chosenSlope = self.slopeValues[i_chosenSlope]

        
        rotationMatrix = np.array([[np.cos(chosenOrientation), -np.sin(chosenOrientation)], 
                                   [np.sin(chosenOrientation), np.cos(chosenOrientation)]])

        self.directionValues = np.matmul(rotationMatrix, self.directionValues)

        nextPosition[:2, 0] = currentPosition[:2, 0] + chosenDirection[:,0]
        nextPosition[-1, 0] = currentPosition[-1, 0] + chosenSlope

        nextOrientation[:2, 0] = np.matmul(rotationMatrix, currentOrientation[:2, 0])
        nextOrientation = nextOrientation / np.linalg.norm(nextOrientation)
        #print(nextOrientation)

        return np.round(np.concatenate([nextPosition, nextOrientation], 1), decimals=4)

    def wreckClick(self):
        if self.wreckEnabled == True:
            del self.Poses[-1]
            
            backRotation = - self.Turns[-1]
            del self.Turns[-1]

            rotationMatrix = np.array([[np.cos(backRotation), -np.sin(backRotation)], 
                                   [np.sin(backRotation), np.cos(backRotation)]])
                
            self.directionValues = np.matmul(rotationMatrix, self.directionValues)

            self.delete_gate_request()

            print("Current Pose:\n" + str(self.Poses[-1]))
            
            if len(self.Poses) == 1:
                self.wreckEnabled = False
                self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.SUNKEN)

                self.trajEnabled = False
                self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.SUNKEN)

            self.buildEnabled = True
            self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.RAISED)
        else:
            print("No Gate built yet.")


    def spawn_gate_request(self):

        model_name = 'Gate' + format(len(self.Poses) - 1, '03')
        
        f = open(os.path.abspath(os.path.join(dirname,'../models/gate/model.sdf')),'r')
        model_xml = f.read() # Syntax

        robot_namespace = 'racetrack'
        
        x = self.Poses[-1][0, 0]
        y = self.Poses[-1][1, 0]
        z = self.Poses[-1][2, 0]
        position = Point(x, y, z)

        xx, yy, zz, ww = self.as_quaternion()

        #xx = 0
        #yy = 0
        #zz = 0
        #ww = 0
        orientation = Quaternion(xx, yy, zz, ww)
        initial_pose = Pose(position, orientation)
        '''
        Or:
        initial_pose = Pose()
        initial_pose.position.x = 1
        initial_pose.position.y = 1
        initial_pose.position.z = 1
        ...
        '''
        
        reference_frame = 'world'

        request = SpawnModelRequest(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        
        #print(request)

        response = self.spawn_model_client(request)

    def as_quaternion(self):

        a = np.array([1, 0, 0])
        b = self.Poses[-1][:, 1]
        #print(a.shape, b.shape)
        v = np.cross(a, b)
        #s = np.linalg.norm(v)
        c = np.dot(a, b)
        if not c == - 1:
            skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            Rot = np.eye(3) + skew + np.matmul(skew, skew) / (1+c)
        else:
            Rot = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        r = R.from_dcm(Rot)
        quat = r.as_quat()
        #return [0, 0, 0, 0]
        return [quat[0], quat[1], quat[2], quat[3]]


    def delete_gate_request(self):

        model_name = 'Gate' + format(len(self.Poses), '03')
        
        request = DeleteModelRequest(model_name)

        response = self.delete_model_client(request)


    def spawn_model_client(self, request):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        #rospy.wait_for_service('gazebo_msgs/SpawnModel')
        try:
            spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            print('is there...')
            response = spawn_sdf_model(request)
            print('success')
            return response.success
        except rospy.ServiceException, e:
            print "Spawn call failed: %s"%e


    def delete_model_client(self, request):
        rospy.wait_for_service('gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            response = delete_model(request)
            return response.success
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


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":

    rospy.init_node('spawn_model_client', log_level=rospy.INFO)

    GateDiameter, DirectionValues, SlopeValues = DefineMetrics()

    # *** WINDOW *** #
    root = tk.Tk()
    # *** FRAME *** #
    DroneRacetrackNode = DroneRacetrackNode(root)
    # Keep window continuosly open until close button pressed
    root.mainloop()