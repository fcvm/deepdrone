#!/usr/bin/env python
# -*- coding: utf-8 -*-



'____ TO DO ____'

# - 

'____ \TO DO ____'



'____ IMPORT ____'
# Generic modules
import os; dirname = os.path.dirname(__file__)
import sys
import numpy as np
from scipy.spatial.transform import Rotation
# ROS packages
import rospy
#from ___ import ___, ___, ...
from sensor_msgs.msg import Image
#from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
#from geometry_msgs.msg import Pose, Point, Quaternion
# Own scripts
#from ___ import *
'____ \IMPORT ____'



'____ CLASS ____'
#class Img2Traj( ):
def process( img ):
    print("...")
    pass

def listen( ):
    rospy.init_node('Img2Traj', anonymous=True, log_level=rospy.INFO)
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, process)
    rospy.spin()
'____ \CLASS ____'



'____ MAIN ____'
if __name__ == "__main__":
    listen()
'____ \MAIN ____'