#!/usr/bin/env python

import actionlib
import datetime
import easygui
import rospy
import smach

from actionlib import GoalStatus
from ar_track_alvar.msg import AlvarMarkers
from collections import OrderedDict
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import  pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from rigid import rigid_transform_3D
import numpy as np
# A list of positions and tasks
task_list = {'position1':['read_tag'], 'position2':['read_tag'], 'position3':['read_tag'], 'position4':['read_tag']}


class main():
    def __init__(self):
        rospy.init_node('find_treasure', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        
        
        # Turn the position locations into SMACH move_base action states
        nav_states = {}

        rospy.loginfo("Initializing position")

        init_pose = PoseWithCovarianceStamped()
        init_pose.pose.pose.position.x = 1;
        init_pose.pose.pose.position.z = 0;
        init_pose.pose.pose.position.y = -1;

        init_pose.pose.pose.orientation.x = 0;
        init_pose.pose.pose.orientation.z = 0;
        init_pose.pose.pose.orientation.y = -0.707;
        init_pose.pose.pose.orientation.w = 0.707;

        rate = rospy.Rate(10)
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        self.is_visible = False
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.doSomething)
        self.poseBot = rospy.Subscriber('amcl_pose', 1, amclPose_callback)
        self.is_visible = False;

        while not rospy.is_shutdown():
            pub.publish(init_pose);
            rate.sleep()
        
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #sm_nav.request_preempt()
        
        rospy.sleep(1)


    def amclPose_callback(self, pmsg):
        pmsg.pose.pose



    def doSomething(self, msg):
        try:

            rospy.loginfo("trying")
            rospy.loginfo("tags detected: %d", len(msg.markers))

            if len(msg.markers) > 2:

                Detected = np.empty([0,3])
                
                tagid = np.empty([0,1]) #will be used to store id for sorting
                for tag in (msg.markers):
                    rospy.loginfo("%f, %f, %f",tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z);
                    coord = [tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z]  
                    
                    t_id = tag.id 

                    Detected = np.vstack((Detected, coord))
     
                    tagid = np.vstack((tagid, t_id))
                    rospy.loginfo("tag: %d", tagid)
                    rospy.loginfo("tagging")
            sort_idx = np.argsort(tagid, axis = 0)        
            sorted_detected = np.take(detected, sort_idx, 0)
            np.savetxt('test.out', sorted_detected, delimiter=',') 
            
        except:
            if self.is_visible:
                rospy.loginfo("marker not confirmed!!")
            self.is_visible = False
        return        



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Treasure finding test finished.")