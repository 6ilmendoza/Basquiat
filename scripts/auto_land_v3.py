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


import matplotlib.pyplot as plt
from scipy.optimize import least_squares


###################################

# Publishers and parameters

command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

###################################

class RigidBodies:
	def __init__(self):
		# define mavros fake gps msg, to publish to
		self.lantar = PoseStamped()
		self.lantar.pose.position.x	=0.0
		self.lantar.pose.position.y	=0.0
		self.lantar.pose.position.z	=0.0
		self.lantar.pose.orientation.x	=0.0
		self.lantar.pose.orientation.y	=0.0
		self.lantar.pose.orientation.z	=0.0
		self.lantar.pose.orientation.w	=0.0

		self.mav = PoseStamped()
		self.mav.pose.position.x	=0.0
		self.mav.pose.position.y	=0.0
		self.mav.pose.position.z	=0.0
		self.mav.pose.orientation.x	=0.0
		self.mav.pose.orientation.y	=0.0
		self.mav.pose.orientation.z	=0.0
		self.mav.pose.orientation.w	=0.0

		
		
		# define mocap message to subsribe to
		self.mocap_msg				= RigidBodyArray()
	
		self.mav_rbId 				= 0		
		self.lantar_rbId			= 1
		
		# check if tracking is valid
		self.tracking_valid			= False

	# retrieves mocap topic and update /mavros/mocap/pose msg
	def cb(self, msg):
		if (msg is not None):
			if ( len(msg.bodies) > 0):
				self.tracking_valid=msg.bodies[self.lantar_rbId].tracking_valid


				#self.mav.header.stamp		= rospy.Time.now()
				self.mav.header.frame_id		="world"
				
				self.mav.pose.position.x		=-msg.bodies[self.mav_rbId].pose.position.x	# inverted!!!
				self.mav.pose.position.y		=msg.bodies[self.mav_rbId].pose.position.z
				self.mav.pose.position.z		=msg.bodies[self.mav_rbId].pose.position.y

				self.mav.pose.orientation.x	=-msg.bodies[self.mav_rbId].pose.orientation.x
				self.mav.pose.orientation.y	=msg.bodies[self.mav_rbId].pose.orientation.z
				self.mav.pose.orientation.z	=msg.bodies[self.mav_rbId].pose.orientation.y
				self.mav.pose.orientation.w	=msg.bodies[self.mav_rbId].pose.orientation.w



				self.lantar.header.frame_id		="world"
				
				self.lantar.pose.position.x	=-msg.bodies[self.lantar_rbId].pose.position.x	# inverted!!!
				self.lantar.pose.position.y	=msg.bodies[self.lantar_rbId].pose.position.z
				self.lantar.pose.position.z	=msg.bodies[self.lantar_rbId].pose.position.y

				self.lantar.pose.orientation.x	=-msg.bodies[self.lantar_rbId].pose.orientation.x
				self.lantar.pose.orientation.y	=msg.bodies[self.lantar_rbId].pose.orientation.z
				self.lantar.pose.orientation.z	=msg.bodies[self.lantar_rbId].pose.orientation.y
				self.lantar.pose.orientation.w	=msg.bodies[self.lantar_rbId].pose.orientation.w

class mpc_landing:
	def __init__(self):
		self.flags.landed = 0
		self.curr_pos.mav.x = 0
		self.curr_pos.mav.y = 0		
		self.curr_pos.mav.z = 0

		self.curr_pos.lantar.x = 0
		self.curr_pos.lantar.y = 0
		self.curr_pos.lantar.z = 0

def get_pos_data(data):
	output = np.zeros((3,1),dtype=np.double)
	output[0] = data.pose.position.x
	output[1] = data.pose.position.y
	output[2] = data.pose.position.z

	return np.transpose(output)

def circ_func(c,x_data,y_data):
	return np.array((x_data - c[0])**(2) +(y_data-c[1])**(2) -c[2]**(2))

def circ_fit(x0,x_data,y_data):
	sol = least_squares(circ_func,x0, args=(x_data,y_data),bounds=([-np.inf,-np.inf, 0.0],np.inf))
	return sol
	

def main():
	# node name
	rospy.init_node('auto_land', anonymous=True)
	rate = rospy.Rate(10) # Hz
	ds_fac = 5 #downsample factor. fs = rate/ds_fac [hz]

	# array containing past 5 pos
	pos_record = np.zeros((6,5),dtype=np.double)

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
	#Cycle to downsample the update rate
	kd = 0

	while not rospy.is_shutdown():
	
		if rBody.tracking_valid :
			#print "tracking valid..."
			if not(kd%ds_fac > 0) :
				kd = 0

				pos_record = np.roll(pos_record,1,axis=1)
				pos_record[0:3,0] = get_pos_data(rBody.mav)
				pos_record[3:6,0] = get_pos_data(rBody.lantar)

				circ_est = circ_fit([0,0,2],pos_record[3,:],pos_record[4,:])				
				[h,k,rad] = circ_est.x		
				print "Center: ", h , k
				print "Radius: ", rad
				
				#print "LanTar: ",pos_record[3:6,0],"\n","Mav",pos_record[0:3,0]	
					
				#print "LanTar: ",rBody.lantar.pose.position,"\n","Mav",rBody.mav.pose.position
			
				#setp.position = rBody.mavros_msg.pose.position
				(setp.position.x,setp.position.y,setp.position.z) = (rBody.lantar.pose.position.x,rBody.lantar.pose.position.y,rBody.lantar.pose.position.z + 1.2)
				#command.publish(setp)

		else:
			print "tracking not valid"


		command.publish(setp)
		rate.sleep()
		kd = kd + 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

