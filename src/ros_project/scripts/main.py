#!/usr/bin/env python

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
from math import  pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from pose_estimation import PoseEstimation
from random import randrange
from sensor_msgs.msg import Image, CameraInfo
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from rviz_utils import RVizUtils
from read_tag import ReadTagState
from face_detect import FaceDetectState
from face_recognize import FaceRecognitionState
from patrol import PickWaypoint, NavState
from face_recognition.msg import FaceRecognitionAction, FaceRecognitionGoal, FaceRecognitionActionFeedback, FaceRecognitionActionResult 

class main():
    
    def __init__(self):

        rospy.init_node('find_treasure', anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # How long do we have to get to each waypoint?
        self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
        
        # Initialize the patrol counter
        self.patrol_count = 0
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()
        
        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # Create a list to hold the waypoint poses
        self.waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        # -0.163200, 0.044660, 0.193186
        self.waypoints.append(Pose(Point(0.9579,    1.8710, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(0.7555,    0.0692, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(-0.72511,  0.4952, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(0.167730,  2.18168, 0.0), quaternions[2]))

        position_locations = list()
        position_locations.append(('position1', self.waypoints[0]))
        position_locations.append(('position2', self.waypoints[1]))
        position_locations.append(('position3', self.waypoints[2]))
        position_locations.append(('position4', self.waypoints[3]))

        self.position_locations = OrderedDict(position_locations)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        rospy.loginfo("Starting Tasks")
        
        # Initialize a number of parameters and variables
        # setup_task_environment(self)
        RVizUtils.get_instance().init_primary_waypoint_markers(self.waypoints)

        ''' Create individual state machines for assigning tasks to each position '''

        # Initialize the overall state machine
        self.sm_find_treasure = StateMachine(outcomes=['succeeded','aborted','preempted'])
            
        # Build the find treasure state machine from the nav states and treasure finding states
        with self.sm_find_treasure:
            # First State Machine
            sm_nav = StateMachine(outcomes=['succeeded','aborted','preempted'])
            sm_nav.userdata.waypoints = self.waypoints
            sm_nav.userdata.waypoints_primary_count = len(self.waypoints)
            rospy.loginfo(sm_nav.userdata.waypoints)

            with sm_nav:
                # StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
                #              transitions={'succeeded':'NAV_WAYPOINT','aborted':'','preempted':''},
                #              remapping={'waypoint_out':'patrol_waypoint',
                #                         'waypoints_primary_count':'waypoints_primary_count'})
            
                StateMachine.add('NAV_WAYPOINT', NavState(),
                             transitions={'succeeded':'NAV_WAYPOINT', 
                                          'aborted':'', 
                                          'preempted':''},
                             remapping={'waypoint_in':'patrol_waypoint',
                                        'waypoints_primary_count':'waypoints_primary_count'})

            # Second State Machine
            sm_read_tags = StateMachine(outcomes=['valid','invalid','preempted'])
            sm_read_tags.userdata.waypoints = self.waypoints

            with sm_read_tags:
                StateMachine.add('read_tag', ReadTagState('ar_pose_marker', AlvarMarkers), 
                    transitions={'invalid':'read_tag', 'valid':'read_tag', 'preempted':''})

            # Third State Machine
            sm_detect_faces = StateMachine(outcomes=['valid','invalid','preempted'])

            with sm_detect_faces:
                StateMachine.add('detect_face', FaceDetectState("/camera/rgb/image_color", Image),
                    transitions={'invalid':'detect_face', 'valid':'detect_face', 'preempted':''})

            # Forth State Machine
            sm_recognize_faces = StateMachine(outcomes=['succeeded','aborted','preempted'])

            with sm_recognize_faces:
                StateMachine.add('recognize_face', FaceRecognitionState(), transitions={'succeeded':'', 'aborted':'', 'preempted':''})
                # goal = FaceRecognitionGoal()
                # goal.order_id = 1
                # goal.order_argument = ''
                # StateMachine.add('recognize_face', SimpleActionState('face_recognition',
                #                 FaceRecognitionAction, goal=goal), transitions={'succeeded':'', 'aborted':'', 'preempted':''})

            sm_con = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],default_outcome='succeeded',
            child_termination_cb=self.child_term_cb, outcome_cb=self.out_cb)

            with sm_con:
                Concurrence.add('SM_NAV', sm_nav)
                Concurrence.add('SM_READ_TAGS', sm_read_tags)
                # Concurrence.add('SM_DETECT_FACES', sm_detect_faces)
                Concurrence.add('SM_RECOGNIZE_FACES', sm_recognize_faces)

            sm_estimate_position = StateMachine(outcomes=['succeeded','aborted','preempted'])

            with sm_estimate_position:
                StateMachine.add('estimate_pose', PoseEstimation('ar_pose_marker', AlvarMarkers),
                    transitions={'succeeded':'', 'aborted':'estimate_pose', 'preempted':''})

            StateMachine.add('POSE_ESTIMATE', sm_estimate_position, transitions={'succeeded':'CON','aborted':'CON','preempted':'CON'})
            StateMachine.add('CON',sm_con, transitions={'succeeded':'','aborted':'','preempted':''})
                        
        # Create and start the SMACH introspection server sm_find_treasure
        intro_server = IntrospectionServer('find_treasure', self.sm_find_treasure, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_find_treasure.execute()

        rospy.loginfo('the length now is')
        rospy.loginfo(self.waypoints)

        intro_server.stop()
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.sm_find_treasure.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def out_cb(self,outcome_map):
        return 'succeeded'

    def child_term_cb(self,outcome_map):
        rospy.loginfo(outcome_map)
        if 'SM_NAV' in outcome_map and (outcome_map['SM_NAV'] == 'succeeded' or outcome_map['SM_NAV'] == 'aborted'):
            rospy.loginfo('It visited all points')
            return True

        return False

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Treasure finding test finished.")
