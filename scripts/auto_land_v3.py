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

from helpers import *

import matplotlib.pyplot as plt
import scipy.linalg as sp
from scipy.optimize import least_squares
from cvxopt import matrix, solvers



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


	###Params#####
	vel = 2
	ini_pos = np.matrix([1,2,1])
	Ts = 0.5
	horizon = 10
	offset = np.matrix([0,0,0])
	center = np.matrix([0,0])
	x0 = np.asmatrix(np.zeros(([9,1])))

	[A,B,C] = load_model()
	solvers.options['show_progress'] = False

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
				
				center = np.matrix([h,k])
				ini_pos = np.matrix(pos_record[3:,0]) #Lantar's latest position
				offset = np.matrix(pos_record[0:3,0]) #MAV's latest position 
				y_ref = ref_traj_est(vel, ini_pos,Ts,horizon,offset,center)

				[M,D] = get_M(A,B,C,horizon,x0)

				Aeq = matrix(M)
				beq = matrix(D - M*y_ref)


				dU = dUmat(horizon)

				U_weights = 20*np.identity(3*horizon) + dU.T*np.identity(3*(horizon-1))*dU

				H = sp.block_diag(np.identity(3*horizon),U_weights)
				f = np.zeros((y_ref.size,1))
				
				[cols, rows] = H.shape 

				G = np.zeros((rows,rows))
				h = np.zeros((rows,1))

				H = matrix(H)
				f = matrix(f)
				G = matrix(G)
				h = matrix(h)
				Aeq = matrix(Aeq)
				beq = matrix(beq)

				sol = solvers.qp(H,f, G, h, Aeq, beq)

				if (sol['status'] == 'optimal'):
					act = sol['x']
					act = act[3*horizon:]
					act = np.reshape(act,(-1,3))

					new_act = np.asmatrix(act[0,:])

					#print A.shape,x0.shape,B.shape,new_act.shape
					x0 = A*x0 + B*new_act.T
					print "success"
					print " Act: ", new_act

				
				#print "LanTar: ",pos_record[3:6,0],"\n","Mav",pos_record[0:3,0]	
					
				#print "LanTar: ",rBody.lantar.pose.position,"\n","Mav",rBody.mav.pose.position
			
				#setp.position = rBody.mavros_msg.pose.position
				#(setp.position.x,setp.position.y,setp.position.z) = (rBody.lantar.pose.position.x,rBody.lantar.pose.position.y,rBody.lantar.pose.position.z + 1.2)
				(setp.position.x,setp.position.y,setp.position.z) = (new_act[0,0],new_act[0,1],-new_act[0,2])
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

