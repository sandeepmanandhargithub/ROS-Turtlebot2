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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State

class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                        input_keys=['waypoints', 'waypoints_primary_count'], 
                        output_keys=['waypoint_out', 'waypoints_primary_count'])
    
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        waypoint_out = 0
        
        userdata.waypoint_out = waypoint_out
        
        rospy.loginfo("Going to waypoint " + str(waypoint_out))
    
        return 'succeeded'

class NavState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                        input_keys=['waypoints', 'waypoint_in', 'waypoints_primary_count'],
                        output_keys=['waypoints_primary_count'])
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if len(userdata.waypoints) == 0:
            return 'aborted'

        self.goal.target_pose.pose = userdata.waypoints[0]
        print('target goal')
        print(self.goal.target_pose.pose)
    
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            # return 'aborted'
        else:        
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                userdata.waypoints.remove(userdata.waypoints[0])
                userdata.waypoints_primary_count = userdata.waypoints_primary_count - 1
                rospy.loginfo("Goal succeeded!")
        
        return 'succeeded'
