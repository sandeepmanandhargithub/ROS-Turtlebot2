#!/usr/bin/env python

import roslib; roslib.load_manifest('joy2twist')
import rospy
from sensor_msgs.msg import Joy 

def callback(data):
	rospy.loginfo(rospy.get_name() + \
		": Get from Joy [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]" %data.axes)

def listener():

	#initialize a node as listener

	rospy.init_node('listener', anonymous=True)

	#subsribe to topic /joy
	rospy.Subscriber("/joy", Joy, callback)

	#keep going until node is stopped
	rospy.spin() 


if __name__ == '__main__':
	listener()