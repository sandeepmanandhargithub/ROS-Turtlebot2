import actionlib
import cv2
import cv2.cv as cv
import datetime
import easygui
import numpy as np 
import rospy
import smach
import threading

from actionlib import GoalStatus
from ar_track_alvar.msg import AlvarMarkers
from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import  pi, sqrt, atan2, asin
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from random import randrange
from sensor_msgs.msg import Image, CameraInfo
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from rviz_utils import RVizUtils
from sound_utils import SoundUtils

class ArTag():
    def __init__(self, x,y,z, q1, q2, q3, q0, id):
        self.id = id

        self.pose = {}
        self.pose['x'] = x
        self.pose['y'] = y
        self.pose['z'] = z

        self.orientation = {}
        self.orientation['x'] = q1
        self.orientation['y'] = q2
        self.orientation['z'] = q3
        self.orientation['w'] = q0


class ReadTagState(MonitorState):
    def __init__(self, topic, msg_type, max_checks=-1):
        smach.State.__init__(self, outcomes=['valid', 'invalid', 'preempted'],
                input_keys=['waypoints'], output_keys=['waypoints'])

        self._topic = topic 
        self._msg_type = msg_type 
        self._cond_cb = self.registerAR_cb 
        #self._cond_cb = self.execute_cb 
        self._max_checks = max_checks 
        self._n_checks = 0 
        self.is_visible = False
        self._trigger_cond = threading.Condition()
        self.arList = []
        self.task = 'read_tag'
        self.added1 = False
        self.added2 = False

    def registerAR_cb(self, userdata, msg): #userdata necessary for statemachine
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        try:
            marker = msg.markers[0]
            if not self.is_visible:
                rospy.loginfo("tags detected: %d", len(msg.markers))
                # rospy.loginfo("marker detected!!")

                self.is_visible = True;
                tX = marker.pose.pose.position.x
                tY = marker.pose.pose.position.y
                tZ = marker.pose.pose.position.z
                rospy.loginfo("ok let's check")
                if len(self.arList) > 0:
                    rospy.loginfo("checking for valid tag")
                    #rospy.loginfo("self.arList[0].x :%f",self.arList[0].pose['x'])
                    for i in range(len(self.arList)):
                        # if sqrt((self.arList[i].pose['x'] - tX)**2 + (self.arList[i].pose['y'] - tY)**2 + (self.arList[i].pose['z'] - tZ)**2) < 0.05:
                        if self.arList[i].id == marker.id:
                            rospy.loginfo("distance from prev: %f", sqrt((self.arList[i].pose['x'] - tX)**2 + (self.arList[i].pose['y'] - tY)**2 + (self.arList[i].pose['z'] - tZ)**2))
                            #preventing from relocalising the target position
                            return 'invalid'

                w = marker.pose.pose.orientation.w         #self.arList[i].orientation['w']
                i = marker.pose.pose.orientation.x         #self.arList[i].orientation['x']
                j = marker.pose.pose.orientation.y         #self.arList[i].orientation['y']
                k = marker.pose.pose.orientation.z 

                yaw = atan2(2*j*w - 2*i*k, 1 - 2*j*j - 2*k*k)*180/pi
                pitch = asin(2*i*j + 2*k*w)*180/pi
                roll = atan2(2*i*w - 2*j*k, 1 - 2*i*i - 2*k*k)*180/pi

                if yaw < 110 and yaw > 70 and roll > -10 and roll < 10:
                    dx = 0.5; dy = 0 
                    angle = 0
                    rospy.loginfo("!!tag faceing along x-axis!!")
                elif yaw < -70 and yaw > -110 and roll > -10 and roll < 10:
                    dx = -0.5; dy = 0
                    # q = Quaternion(0, 0, 0.7071, 0.7071)
                    angle = pi
                    rospy.loginfo("!!tag faceing against x-axis!!")
                elif yaw < 110 and yaw > 70 and roll < -70 and roll > -110:
                    dx = 0; dy = 0.5
                    # q = Quaternion(0, 0, 0.7071, 0.7071)
                    angle = 3*pi/2
                    rospy.loginfo("!!tag faceing along y-axis!!")
                elif yaw < 110 and yaw > 70 and roll < 110 and roll > 70:
                    dx = 0; dy = -0.5
                    angle = pi/2
                x_location = tX + dx
                y_location = tY + dy

                q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
                quaternion = Quaternion(*q_angle)

                rospy.loginfo("appending tag: %d ",marker.id)
                self.arList.append(ArTag(tX, tY, tZ, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w, marker.id )) 
                rospy.loginfo( "NEW length of list: %d", len(self.arList) )       
                rospy.loginfo("length of goal points: %f", len(userdata.waypoints))
                # convert to Quaternion
                pose = Pose(Point(x_location, y_location, 0.0), quaternion)
                userdata.waypoints.append(pose)
                RVizUtils.get_instance().init_secondary_waypoint_markers(pose)
                sound_message = "New Tag " + str(marker.id) + " is registered"
                rospy.loginfo(sound_message)
                SoundUtils.get_instance().say_message("New Tag " + str(marker.id) + " is registered")
                # rospy.loginfo("length of goal points: %f", len(userdata.waypoints))
        
        except:
            # if self.is_visible:
                # rospy.loginfo("marker not confirmed!!")
                
            self.is_visible = False
            return 'invalid'
        return 'valid'

    # Callback for testing purpose on simulation
    def execute_cb(self, userdata, msg):
        if self.preempt_requested():
            print "state foo is being preempted!!!"
            self.service_preempt()
            return 'preempted'

        if not self.added1:
            self.added1 = True
            pose = Pose(Point(2.56839, 1.918, 0.0), Quaternion(0,0,0.707106781187,0.707106781187))
            userdata.waypoints.append(pose)
            print len(userdata.waypoints)
            RVizUtils.get_instance().init_secondary_waypoint_markers(pose)

        if self.added1 and (not self.added2):
            self.added2 = True
            print len(userdata.waypoints)
            pose = Pose(Point(0.957999, 2.54, 0.0), Quaternion(0,0,0.707106781187,0.707106781187))
            userdata.waypoints.append(pose)
            RVizUtils.get_instance().init_secondary_waypoint_markers(pose)


        rospy.loginfo('I am reading the tags and updating the list')
        rospy.sleep(5)

        return 'succeeded'
