#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

import numpy as np


class OffbGlobalTraj():

	def __init__( self, t, x, y, z ):

		# Initial Pose of drone
		self.pose = PoseStamped()
		pose.pose.position.x = x[ 0 ]
		pose.pose.position.y = y[ 0 ]
		pose.pose.position.z = z[ 0 ]
	
		# Rate
		self.rate = rospy.Rate(20)

		# Subscribers
		rospy.Subscriber(
			'mavros/state', 
			State, 
			callback=self.Callback
		)

		# Publishers
		local_pos_pub = rospy.Publisher(
			'mavros/setpoint_position/local', 
			PoseStamped, 
			queue_size=10
		)

		arming_client = rospy.ServiceProxy(
			'mavros/cmd/arming', 
			CommandBool
		)
		
		set_mode_client = rospy.ServiceProxy(
			'mavros/set_mode', 
			SetMode
		)

	def Callback( self, msg ):
		self.state = msg
		rospy.loginfo("state: {}".format(self.state))

	def Start( self ):

		while( not self.state.connected ):
			print( self.state.connected )
			rate.sleep()

		for i in range( 100 ):
			local_pos_pub.publish( pose )
			rate.sleep()

		offb_set_mode = SetMode()
		offb_set_mode.custom_mode = "OFFBOARD"

		arm_cmd = CommandBool()
		arm_cmd.value = True

		last_request = rospy.Time.now()

		while not rospy.is_shutdown():
			if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
				resp1 = set_mode_client(0,offb_set_mode.custom_mode)
				if resp1.mode_sent:
					print ("Offboard enabled")
				last_request = rospy.Time.now()
			elif (not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
				arm_client_1 = arming_client(arm_cmd.value)
				if arm_client_1.success:
					print("Vehicle armed")
				last_request = rospy.Time.now()
				
			local_pos_pub.publish(pose)
			#local_vel_pub.publish(vel)
			#print current_state
			rate.sleep()


		#################
        rospy.loginfo("In attesa")

        while not rospy.is_shutdown():
            for ii in xrange(1, 51):
                self.pub.publish(self.x1)
                self.loop_rate.sleep()





	













if __name__ == "__main__" :

	'''
	Write a service to get the trajectory data here
	'''
	# Wait that trajectory was calculated
	#rospy.wait_for_service('gazebo/spawn_sdf_model')
	# Mockup Data
	t = np.arange(0, 10, 0.1)
	x = np.arange(0, 10, 0.1)
	y = np.sin(x)
	z = x / 10

	# NODE

	rospy.init_node(
		'OffboardGlobalTrajectoryClient', 
		anonymous=True
	)
	offb_global_traj = OffbGlobalTraj( t, x, y, z )
	offb_global_traj.Start()

	################
	try:
		spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
		response = spawn_sdf_model(request)
		print '\nGate spawned: ', response.success
	except rospy.ServiceException, e:
		print "Spawn call failed: %s"%e

