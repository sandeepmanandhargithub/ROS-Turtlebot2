#!/usr/bin/env python

import actionlib
import cv2
import cv2.cv as cv
import datetime
import easygui
import numpy as np 
import rospy

from actionlib import GoalStatus
from ar_track_alvar.msg import AlvarMarkers
from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import  pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from random import randrange
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

class RVizUtils():

    INSTANCE = None

    @classmethod
    def get_instance(cls):
        if cls.INSTANCE is None:
            cls.INSTANCE = RVizUtils()
        return cls.INSTANCE

    def __init__(self):
        if self.INSTANCE is not None:
            raise ValueError("An instantiation already exists!")

        self.marker_scale = 0.2
        self.marker_lifetime = 0 # 0 is forever
        self.primary_marker_color = {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}
        self.primary_marker_ns = 'primarywaypoints'
        self.primary_marker_id = 0

        self.primary_marker_pub = rospy.Publisher('waypoint_markers', Marker)

        self.primary_waypoint_markers = Marker()
        self.primary_waypoint_markers.ns = self.primary_marker_ns
        self.primary_waypoint_markers.id = self.primary_marker_id
        self.primary_waypoint_markers.type = Marker.CUBE_LIST
        self.primary_waypoint_markers.action = Marker.ADD
        self.primary_waypoint_markers.lifetime = rospy.Duration(self.marker_lifetime)
        self.primary_waypoint_markers.scale.x = self.marker_scale
        self.primary_waypoint_markers.scale.y = self.marker_scale
        self.primary_waypoint_markers.color.r = self.primary_marker_color['r']
        self.primary_waypoint_markers.color.g = self.primary_marker_color['g']
        self.primary_waypoint_markers.color.b = self.primary_marker_color['b']
        self.primary_waypoint_markers.color.a = self.primary_marker_color['a']
        
        self.primary_waypoint_markers.header.frame_id = 'map'
        self.primary_waypoint_markers.header.stamp = rospy.Time.now()
        self.primary_waypoint_markers.points = list()


        self.secondary_marker_color = {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}
        self.secondary_marker_ns = 'secondarywaypoints'
        self.secondary_marker_id = 1
        
        self.secondary_marker_pub = rospy.Publisher('waypoint_markers', Marker)

        self.secondary_waypoint_markers = Marker()
        self.secondary_waypoint_markers.ns = self.secondary_marker_ns
        self.secondary_waypoint_markers.id = self.secondary_marker_id
        self.secondary_waypoint_markers.type = Marker.CUBE_LIST
        self.secondary_waypoint_markers.action = Marker.ADD
        self.secondary_waypoint_markers.lifetime = rospy.Duration(self.marker_lifetime)
        self.secondary_waypoint_markers.scale.x = self.marker_scale
        self.secondary_waypoint_markers.scale.y = self.marker_scale
        self.secondary_waypoint_markers.color.r = self.secondary_marker_color['r']
        self.secondary_waypoint_markers.color.g = self.secondary_marker_color['g']
        self.secondary_waypoint_markers.color.b = self.secondary_marker_color['b']
        self.secondary_waypoint_markers.color.a = self.secondary_marker_color['a']
        
        self.secondary_waypoint_markers.header.frame_id = 'map'
        self.secondary_waypoint_markers.header.stamp = rospy.Time.now()
        self.secondary_waypoint_markers.points = list()

    def init_primary_waypoint_markers(self,primary_waypoints):
        for waypoint in primary_waypoints:           
            p = Point()
            p = waypoint.position
            self.primary_waypoint_markers.points.append(p)

        self.primary_marker_pub.publish(self.primary_waypoint_markers)
        rospy.sleep(1)
        self.primary_marker_pub.publish(self.primary_waypoint_markers)


    def init_secondary_waypoint_markers(self,secondary_waypoint):
        p = Point()
        p = secondary_waypoint.position
        self.secondary_waypoint_markers.points.append(p)
        
        self.secondary_marker_pub.publish(self.secondary_waypoint_markers)
        rospy.sleep(1)
        self.secondary_marker_pub.publish(self.secondary_waypoint_markers)
