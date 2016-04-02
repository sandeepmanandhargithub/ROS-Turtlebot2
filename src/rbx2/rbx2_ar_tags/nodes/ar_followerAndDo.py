#!/usr/bin/env python

"""
    ar_follower.py - Version 1.0 2013-08-25
    
    Follow an AR tag published on the /ar_pose_marker topic.  The /ar_pose_marker topic
    is published by the ar_track_alvar package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import copysign
from math import radians, pi

class ARFollower():
    def __init__(self):
        rospy.init_node("ar_follower")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate) 
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.5)
        
        # The maximum distance a target can be from the robot for us to track
        self.max_x = rospy.get_param("~max_x", 20.0)
        
        # The goal distance (in meters) to keep between the robot and the marker
        self.goal_x = rospy.get_param("~goal_x", 0.6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How far away from being centered (y displacement) on the AR marker
        # before the robot reacts (units are meters)
        self.y_threshold = rospy.get_param("~y_threshold", 0.05)
        
        # How much do we weight the goal distance (x) when making a movement
        self.x_scale = rospy.get_param("~x_scale", 0.5)

        # How much do we weight y-displacement when making a movement        
        self.y_scale = rospy.get_param("~y_scale", 1.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Intialize the movement command
        self.move_cmd = Twist()
        
        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
        
        # Wait for the ar_pose_marker topic to become available
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        
        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)
        
        rospy.loginfo("Marker messages detected. Starting follower...")
        
        """
        Some modification

        """
        self.canFollow = True    
        self.TakeRight = False
        quaternions = list()
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        waypoints = list()
        waypoints.append(Pose(Point(0, 0.0, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(1, 1, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(0.0, 1, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))

        # not using markers

        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
        

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))    
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")   





        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            # Send the Twist command to the robot
            self.cmd_vel_pub.publish(self.move_cmd)
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    def set_cmd_vel(self, msg):
        # Pick off the first marker (in case there is more than one)
        try:
            marker = msg.markers[0]
            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            # If target is loar, stop the robot by slowing it incrementally
            self.move_cmd.linear.x /= 1.5
            self.move_cmd.angular.z /= 1.5
            
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False
            
            return
                
        # Get the displacement of the marker relative to the base
        target_offset_y = marker.pose.pose.position.y
        
        # Get the distance of the marker from the base
        target_offset_x = marker.pose.pose.position.x
        
        # Rotate the robot only if the displacement of the target exceeds the threshold
        if abs(target_offset_y) > self.y_threshold :
            # Set the rotation speed proportional to the displacement of the target
            if self.canFollow == True:
                speed = target_offset_y * self.y_scale
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(speed))), speed)
        else:
            self.move_cmd.angular.z = 0.0
 
        # Now get the linear speed
        if abs(target_offset_x - self.goal_x) > self.x_threshold:
            if self.canFollow == True:
              speed = (target_offset_x - self.goal_x) * self.x_scale
              if speed < 0:
                    speed *= 1.5
                    self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
        else:
            self.move_cmd.linear.x = 0.0
            self.canFollow == False
            TakeRight == True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoint[0]
            self.move(goal)


    def move(self, goal):
        self.move_base.send_goal(goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
               rospy.loginfo("Goal succeeded!")
        

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        ARFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AR follower node terminated.")

