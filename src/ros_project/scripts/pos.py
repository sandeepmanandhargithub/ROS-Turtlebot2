import numpy as np 
from math import cos, sin, pi, sqrt
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped


def getRTfromQuat(translation, quat):

	# pose_map = np.array([[-0.22571, 0.063645, 0.20089]])
	# orient_map = np.array([ -0.0027935, 0.70555, -0.0043037, 0.70864])

	
	# print quat
	[roll1,pitch1,yaw1] = euler_from_quaternion(np.squeeze(np.asarray(quat)))
	# print roll1, pitch1, yaw1
	Rp1=np.matrix([[cos((pitch1)), 0, sin((pitch1))],\
	     		   [0            , 1,             0],\
	     		   [-sin(pitch1) , 0, cos((pitch1))]])


	Ry1=np.matrix([[cos((yaw1)) ,-sin((yaw1)) ,0],\
	     [sin((yaw1)), cos((yaw1)), 0],\
	     [0, 0, 1]]) 

	Rr1=np.matrix([[1 ,0 ,0],\
	    [0 ,cos((roll1)), -sin((roll1))],\
	    [0 ,sin((roll1)) ,cos((roll1))]]) 

	R1=Ry1*Rp1*Rr1

	R = np.asarray(R1)
	R = np.vstack((R, np.array([0,0,0])))


	R = np.hstack((R, np.array([[translation.item(0)],[translation.item(1)],[translation.item(2)],[1]])))
	print R
	return R

def getQuatfromRT(RT):
	qw = sqrt(1+RT[0,0] + RT[1,1]+ RT[2,2])/2
	qx = (RT[2,1] - RT[1,2])/(4*qw)
	qy = (RT[0,2] - RT[2,0])/(4*qw)
	qz = (RT[1,0] - RT[0,1])/(4*qw)
	return np.array([qx, qy, qz, qw])

# def getPOSE(poseODOM_ar3, orientODOM_ar3):

"""
	Ground truth about ar tag pose_map
"""
# poseAR_map = np.array([[-0.22571, 0.063645, 0.20089]])
# orientAR_map = np.array([ -0.0027935, 0.70555, -0.0043037, 0.70864])


# 
# poseAR_map = np.array([[3.3404, 0.516725, 0.344577]])
# orientAR_map = np.array([-0.0118515, -0.72394, 0.0075599, 0.68972])

poseAR_map = np.array([[3.3183, 0.43744, 0.36668]])
orientAR_map = np.array([-0.034095, -0.69846, 0.027996, 0.71428])


"""
	Robot pose wrt to tag in disoriented map 
"""


# poseODOM_ar3 = np.array([0.21105, 0.13398, 1.736]);
# orientODOM_ar3 = np.array([ -0.0312144, -0.704887, 0.0224505, 0.708277])

# poseODOM_ar3 = np.array([[-0.32219, 0.18232, 1.1379]])
# orientODOM_ar3 = np.array([0.052972, 0.71198, -0.0866, 0.696344])
def getPOSE(poseODOM_ar3, orientODOM_ar3 ):
	T_MA = np.asmatrix(getRTfromQuat(poseAR_map, orientAR_map))
	T_AO = np.asmatrix(getRTfromQuat(poseODOM_ar3, orientODOM_ar3))

	T_MO = T_MA*T_AO
	Rr = T_MO[0:3, 0:3]
	position = T_MO[0:3, -1]  #supress the z coordinate
	orientation = getQuatfromRT(T_MO)
	print "position"
	print position
	print "orientation"
	print orientation
	return position, orientation


	# initPose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=20)
	# init_pose = PoseWithCovarianceStamped()
	# init_pose.pose.pose.position.x = position.item(0)
	# init_pose.pose.pose.position.y = position.item(1)
	# init_pose.pose.pose.position.z = 0;

	# init_pose.pose.pose.orientation.x = orientation[0]
	# init_pose.pose.pose.orientation.y = orientation[1]
	# init_pose.pose.pose.orientation.z = orientation[2]
	# init_pose.pose.pose.orientation.w = orientation[3]


	# rospy.loginfo("publishing pose")
	# initPose_pub.publish(init_pose)
	# rospy.sleep(0.1)