#!/usr/bin/env python
import rospy
import numpy as np
import tf

#from std_msgs.msg import *
#from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from optitrack.msg import RigidBody, RigidBodyArray

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

###################################

# Publishers and parameters

command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

###################################

class RigidBodies:
	def __init__(self):
		# define mavros fake gps msg, to publish to
		self.mavros_msg = PoseStamped()
		self.mavros_msg.pose.position.x		=0.0
		self.mavros_msg.pose.position.y		=0.0
		self.mavros_msg.pose.position.z		=0.0
		self.mavros_msg.pose.orientation.x	=0.0
		self.mavros_msg.pose.orientation.y	=0.0
		self.mavros_msg.pose.orientation.z	=0.0
		self.mavros_msg.pose.orientation.w	=0.0
		
		# define mocap message to subsribe to
		self.mocap_msg				= RigidBodyArray()
	
		self.rbId				= 1
		
		# check if tracking is valid
		self.tracking_valid			= False

	# retrieves mocap topic and update /mavros/mocap/pose msg
	def cb(self, msg):
		if (msg is not None):
			if ( len(msg.bodies) > 0):
				self.tracking_valid=msg.bodies[self.rbId].tracking_valid

				#self.mavros_msg.header.stamp		= rospy.Time.now()
				self.mavros_msg.header.frame_id		="world"
				
				self.mavros_msg.pose.position.x		=-msg.bodies[self.rbId].pose.position.x	# inverted!!!
				self.mavros_msg.pose.position.y		=msg.bodies[self.rbId].pose.position.z
				self.mavros_msg.pose.position.z		=msg.bodies[self.rbId].pose.position.y

				self.mavros_msg.pose.orientation.x	=-msg.bodies[self.rbId].pose.orientation.x
				self.mavros_msg.pose.orientation.y	=msg.bodies[self.rbId].pose.orientation.z
				self.mavros_msg.pose.orientation.z	=msg.bodies[self.rbId].pose.orientation.y
				self.mavros_msg.pose.orientation.w	=msg.bodies[self.rbId].pose.orientation.w

def main():
	# node name
	rospy.init_node('auto_land', anonymous=True)
	rate = rospy.Rate(10) # Hz

        # Instantiate a setpoint
        setp = PositionTarget()
        setp.type_mask = int('011111111000', 2)

	# instantiate a rigid body object
	rBody = RigidBodies()
	
	# subscribe to mocap topic
	rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, rBody.cb)

	# Cycle to register local position
        kc = 0.0
        while kc < 10: # cycle for subscribers to read local position
        	rate.sleep()
        	kc = kc + 1

	while not rospy.is_shutdown():
		if rBody.tracking_valid :
			#print "tracking valid..."
			rBody.mavros_msg.header.stamp= rospy.Time.now()
			#mavros_pub.publish(rBody.mavros_msg)
			print "LanTar: ",rBody.mavros_msg.pose.position
			
			#setp.position = rBody.mavros_msg.pose.position
			(setp.position.x,setp.position.y,setp.position.z) = (rBody.mavros_msg.pose.position.x,rBody.mavros_msg.pose.position.y,rBody.mavros_msg.pose.position.z + 1.2)
			command.publish(setp)
		else:
			print "tracking not valid"
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
