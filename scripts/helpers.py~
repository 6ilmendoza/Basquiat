import numpy as np
import math as m

def get_M(A,B,C,horizon,x0):
	#print "starting"
	import numpy as np
	[rows,cols] = B.shape
	[C_rows,C_cols] = C.shape
	cab = np.asmatrix(np.empty([C_rows,0]))
	CAB = np.asmatrix(np.empty([0,(horizon)*C_rows]))
	D = np.asmatrix(np.empty([0,1]))
	
	for i in range(1,horizon+1):
		#print "i= ",i
		cab = np.asmatrix(np.empty([C_rows,0]))
		
		for k in range(1,i+1):
			#print "k= ", k
			cab = np.concatenate((cab, -C*A**(i-k)*B),axis=1)
		#print "cab.shape: ", cab.shape
		cab = np.concatenate((cab,np.zeros([C_rows,(horizon-i)*cols])),axis=1)
		#print "cab.shape: ", cab.shape
		#print "CAB.shape: ",CAB.shape                
		CAB = np.concatenate((CAB,cab),axis=0)
		
		D = np.concatenate((D,C*A**(i)*x0),axis=0)

	[CAB_rows,CAB_cols] = CAB.shape
	M = np.concatenate((np.asmatrix(np.identity(CAB_rows)),CAB),axis=1)
	
	return [M,D]


#vel = 2
#ini_pos = np.matrix([1,2,1])
#Ts = 0.5
#horizon = 10
#offset = np.matrix([0,0,0])
#center = np.matrix([0,0])

def ref_traj_est(vel, ini_pos,Ts,horizon,offset,center):
	
	time_hor = np.asmatrix(np.arange(0,horizon*Ts,Ts))
	#cartesian to polar conversion
	[x0,y0] = [ini_pos[0,0]-center[0,0],ini_pos[0,1]-center[0,1]]
		
	rad = np.sqrt(x0**2 + y0**2)
	ini_theta = np.arctan2(y0,x0)
	f_theta = -vel/rad*time_hor + ini_theta

	#polar to cartesian
	x = rad*np.cos(f_theta)
	y = rad*np.sin(f_theta)
	z = ini_pos[0,2] + 0.16 # check convetion for positive height

	x = x - offset[0,0]*np.ones([1,x.size]) + center[0,0]
	y = y - offset[0,1]*np.ones([1,y.size]) + center[0,1]
	z = z - offset[0,2]*np.ones([1,z.size]) 

	pos = np.concatenate((x,y,z*np.ones(horizon)),axis=0).T.reshape((3*horizon,1))
	pos = np.concatenate((pos,np.zeros([3*horizon,1])),axis=0)        
	
	return pos

def dUmat(horizon):
	import numpy as np

	p_ones = np.ones((3*horizon))
	n_ones = -np.ones((3*(horizon-1)))
	deltaUmat1 = np.diag(p_ones, k =0)
	deltaUmat2 = np.diag(n_ones, k =-3)
	deltaUmat = deltaUmat1 + deltaUmat2
	deltaUmat = deltaUmat[3:,:]

	return np.asmatrix(deltaUmat)

def get_pos_data(data):
	output = np.zeros((3,1),dtype=np.double)
	output[0] = data.pose.position.x
	output[1] = data.pose.position.y
	output[2] = data.pose.position.z

	return np.transpose(output)

def circ_func(c,x_data,y_data):
	return np.array((x_data - c[0])**(2) +(y_data-c[1])**(2) -c[2]**(2))

def circ_fit(x0,x_data,y_data):
	from scipy.optimize import least_squares

	sol = least_squares(circ_func,x0, args=(x_data,y_data),bounds=([-np.inf,-np.inf, 0.0],np.inf))
	return sol

def load_model():

	A = np.asmatrix(np.zeros((9,9)))
	B = np.asmatrix(np.zeros((9,3)))
	C = np.asmatrix(np.zeros((3,9)))

	A[0:3,0:3] = np.matrix([[0.7305,0.6224,0.0305],[-0.3169,0.2278,0.9077],[-0.0684,-0.0096,-0.0916]])
	A[3:6,3:6] = np.matrix([[0.7163,-0.6181,-0.0693],[0.3540,0.1652,0.9091],[0.082,-0.0291,0.072]])
	A[6:9,6:9] = np.matrix([[0.6982,0.687,0.1774],[-0.1674,-0.0767,0.7948],[0.0039,0.018,0.9249]])


	B[0:3,0] = np.matrix([[0.5217],[0.4189],[0.0947]])
	B[3:6,1] = np.matrix([[0.5373],[-0.4051],[-0.1234]])
	B[6:9,2] = np.matrix([[-0.47],[-0.237],[0.0105]])


	C[0,0:3] = np.matrix([0.5676,-0.7110,0.4834])
	C[1,3:6] = np.matrix([0.5507,0.7347,-0.4715])
	C[2,6:9] = np.matrix([-0.69,0.7194,-0.0721])

	return [A,B,C]
