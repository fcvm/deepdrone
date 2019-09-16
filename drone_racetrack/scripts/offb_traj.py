#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

import numpy as np


class OffbTraj():

	def __init__( self, t, x, y, z ):

		self.t = t
		self.x = x
		self.y = y
		self.z = z

		# Initial Pose of drone
		self.pose = PoseStamped()
		self.pose.pose.position.x = x[ 0 ]
		self.pose.pose.position.y = y[ 0 ]
		self.pose.pose.position.z = z[ 0 ]
	
		# Rate
		self.rate = rospy.Rate(20)

		# Subscribers
		rospy.Subscriber(
			'mavros/state', 
			State, 
			callback=self.Callback
		)
		self.state = None

		# Publishers
		self.localPosPub = rospy.Publisher(
			'mavros/setpoint_position/local', 
			PoseStamped, 
			queue_size=10
		)

		self.armingClient = rospy.ServiceProxy(
			'mavros/cmd/arming', 
			CommandBool
		)
		
		self.setModeClient = rospy.ServiceProxy(
			'mavros/set_mode', 
			SetMode
		)

	def Callback( self, msg ):
		self.state = msg
		rospy.loginfo("state: {}".format(self.state))

	def Start( self ):

		while(self.state == None):
			print("First receive a state message.")
			self.rate.sleep()

		while( not self.state.connected ):
			print( self.state.connected )
			self.rate.sleep()

		for i in range( 100 ):
			self.localPosPub.publish( self.pose )
			self.rate.sleep()

		offb_set_mode = SetMode()
		offb_set_mode.custom_mode = "OFFBOARD"

		arm_cmd = CommandBool()
		arm_cmd.value = True

		last_request = rospy.Time.now()

		while not rospy.is_shutdown():
			if( self.state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
				resp1 = self.setModeClient( 0, offb_set_mode.custom_mode )
				if resp1.mode_sent:
					print ("Offboard enabled")
				last_request = rospy.Time.now()
			elif (not self.state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
				arm_client_1 = self.armingClient(arm_cmd.value)
				if arm_client_1.success:
					print("Vehicle armed")
				last_request = rospy.Time.now()
				
			startTimeTraj = rospy.Time.now().to_sec()

			while( rospy.Time.now().to_sec() - startTimeTraj <= self.t[ -1 ] ):
				currIndex = np.argmax( self.t >= rospy.Time.now().to_sec() - startTimeTraj )
				self.pose.pose.position.x = x[ currIndex ]
				self.pose.pose.position.y = y[ currIndex ]
				self.pose.pose.position.z = z[ currIndex ]
				self.localPosPub.publish( self.pose )
				#local_vel_pub.publish(vel)
				#print current_state
				self.rate.sleep()





	



if __name__ == "__main__" :

	'''
	Write a service to get the trajectory data here
		################
	try:
		spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
		response = spawn_sdf_model(request)
		print '\nGate spawned: ', response.success
	except rospy.ServiceException, e:
		print "Spawn call failed: %s"%e
	'''
	# Wait that trajectory was calculated
	#rospy.wait_for_service('gazebo/spawn_sdf_model')
	
	
	
	
	
	# Mockup Data
	t = np.arange(0, 10, 1)
	x = np.arange(0, 10, 1)
	y = np.sin(x)
	z = x / 10

	# NODE

	rospy.init_node(
		'OffboardTrajectoryClient', 
		anonymous=True
	)
	offbTraj = OffbTraj( t, x, y, z )
	offbTraj.Start()



