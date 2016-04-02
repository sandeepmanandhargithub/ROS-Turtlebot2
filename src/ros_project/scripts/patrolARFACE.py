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
from random import randrange
from sensor_msgs.msg import Image, CameraInfo
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import atan2, asin, sqrt, pi

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

class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                        input_keys=['waypoints', 'waypoints_primary_count'], 
                        output_keys=['waypoint_out', 'waypoints_primary_count'])
    
    def execute(self, userdata):
        if len(userdata.waypoints) == 0:
            return 'aborted'

        waypoint_out = 0
        
        userdata.waypoint_out = waypoint_out
        
        rospy.loginfo("Going to waypoint " + str(waypoint_out))
    
        return 'succeeded'

class Nav2Waypoint(State):
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
        self.goal.target_pose.pose = userdata.waypoints[userdata.waypoint_in]
        print('target goal')
        print(self.goal.target_pose.pose)
    
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:        
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                userdata.waypoints.remove(userdata.waypoints[userdata.waypoint_in])
                userdata.waypoints_primary_count = userdata.waypoints_primary_count - 1
                rospy.loginfo("Goal succeeded!")
            return 'succeeded'

class ReadTagState(MonitorState):
    def __init__(self, topic, msg_type, max_checks=-1):
        smach.State.__init__(self, outcomes=['valid', 'invalid', 'preempted'],
                input_keys=['waypoints'], output_keys=['waypoints'])

        self._topic = topic 
        self._msg_type = msg_type 
        self._cond_cb = self.registerAR_cb 
        self._max_checks = max_checks 
        self._n_checks = 0 
        self.is_visible = False
        self._trigger_cond = threading.Condition()
        self.arList = []
        self.task = 'read_tag'
        self.added1 = False
        self.added2 = False

    def registerAR_cb(self, userdata, msg): #userdata necessary for statemachine
        try:
            rospy.loginfo("tags detected: %d", len(msg.markers))
            marker = msg.markers[0]
            if not self.is_visible:
                # rospy.loginfo("marker detected!!")

                self.is_visible = True;
                tX = marker.pose.pose.position.x
                tY = marker.pose.pose.position.y
                tZ = marker.pose.pose.position.z
  
                if len(self.arList) > 0:
                    # rospy.loginfo("checking for valid tag")
                    # rospy.loginfo("self.arList[0].x :%f",self.arList[0].pose['x'])
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
                rospy.loginfo("appending tag: %d ",marker.id)
                self.arList.append(ArTag(tX, tY, tZ, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w, marker.id )) 
                rospy.loginfo( "NEW length of list: %d", len(self.arList) )       
                rospy.loginfo("length of goal points: %f", len(userdata.waypoints))
                # convert to Quaternion
                userdata.waypoints.append(Pose(Point(x_location, y_location, 0.0), Quaternion(0,0,0.707106781187,0.707106781187)))
                # rospy.loginfo("length of goal points: %f", len(userdata.waypoints))
       
        except:
            # if self.is_visible:
                # rospy.loginfo("marker not confirmed!!")
                
            self.is_visible = False
            return 'invalid'
        return 'valid'        

    def execute_cb(self, userdata, msg):
        if self.preempt_requested():
            print "state foo is being preempted!!!"
            self.service_preempt()
            return 'preempted'

        if not self.added1:
            self.added1 = True
            print len(userdata.waypoints)
            userdata.waypoints.append(Pose(Point(2.56839, 1.918, 0.0), Quaternion(0,0,0.707106781187,0.707106781187)))

        if self.added1 and (not self.added2):
            self.added2 = True
            print len(userdata.waypoints)
            userdata.waypoints.append(Pose(Point(0.957999, 2.54, 0.0), Quaternion(0,0,0.707106781187,0.707106781187)))


        rospy.loginfo('I am reading the tags and updating the list')
        rospy.sleep(5)

        return 'succeeded'


def setup_task_environment(self):
    # How long do we have to get to each waypoint?
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
    
    # Initialize the patrol counter
    self.patrol_count = 0
    
    # Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
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
    self.waypoints.append(Pose(Point(-0.72511,  0.4952, 0.0), quaternions[0]))
    self.waypoints.append(Pose(Point(0.7555,    0.0692, 0.0), quaternions[1]))
    self.waypoints.append(Pose(Point(0.167730,  2.18168, 0.0), quaternions[2]))

    position_locations = list()
    position_locations.append(('position1', self.waypoints[0]))
    position_locations.append(('position2', self.waypoints[1]))
    position_locations.append(('position3', self.waypoints[2]))
    position_locations.append(('position4', self.waypoints[3]))

    self.position_locations = OrderedDict(position_locations)

    # Where is the docking station?
    self.docking_station_pose = (Pose(Point(0.5, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))            
    
    # Initialize the waypoint visualization markers for RViz
    init_waypoint_markers(self)
    
    # Set a visualization marker at each waypoint        
    for waypoint in self.waypoints:           
        p = Point()
        p = waypoint.position
        self.waypoint_markers.points.append(p)
        
    # Set a marker for the docking station
    init_docking_station_marker(self)
        
    # Publisher to manually control the robot (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
    
    rospy.loginfo("Starting Tasks")
    
    # Publish the waypoint markers
    self.marker_pub.publish(self.waypoint_markers)
    rospy.sleep(1)
    self.marker_pub.publish(self.waypoint_markers)
    
    # Publish the docking station marker
    self.docking_station_marker_pub.publish(self.docking_station_marker)
    rospy.sleep(1)

def init_waypoint_markers(self):
    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
    
    # Define a marker publisher.
    self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
    
    # Initialize the marker points list.
    self.waypoint_markers = Marker()
    self.waypoint_markers.ns = marker_ns
    self.waypoint_markers.id = marker_id
    self.waypoint_markers.type = Marker.CUBE_LIST
    self.waypoint_markers.action = Marker.ADD
    self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
    self.waypoint_markers.scale.x = marker_scale
    self.waypoint_markers.scale.y = marker_scale
    self.waypoint_markers.color.r = marker_color['r']
    self.waypoint_markers.color.g = marker_color['g']
    self.waypoint_markers.color.b = marker_color['b']
    self.waypoint_markers.color.a = marker_color['a']
    
    self.waypoint_markers.header.frame_id = 'map'
    self.waypoint_markers.header.stamp = rospy.Time.now()
    self.waypoint_markers.points = list()

def init_docking_station_marker(self):
    # Define a marker for the charging station
    marker_scale = 0.3
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}
    
    self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker)
    
    self.docking_station_marker = Marker()
    self.docking_station_marker.ns = marker_ns
    self.docking_station_marker.id = marker_id
    self.docking_station_marker.type = Marker.CYLINDER
    self.docking_station_marker.action = Marker.ADD
    self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)
    self.docking_station_marker.scale.x = marker_scale
    self.docking_station_marker.scale.y = marker_scale
    self.docking_station_marker.scale.z = 0.02
    self.docking_station_marker.color.r = marker_color['r']
    self.docking_station_marker.color.g = marker_color['g']
    self.docking_station_marker.color.b = marker_color['b']
    self.docking_station_marker.color.a = marker_color['a']
    
    self.docking_station_marker.header.frame_id = 'odom'
    self.docking_station_marker.header.stamp = rospy.Time.now()
    self.docking_station_marker.pose = self.docking_station_pose
        
class main():
    # gets called when ALL child states are terminated
    def out_cb(self,outcome_map):
        return 'succeeded'

    def child_term_cb(self,outcome_map):
        print(outcome_map)
        if outcome_map['SM_NAV'] == 'succeeded' or outcome_map['SM_NAV'] == 'aborted':
            print 'It succeeded'
            return True

        return False

    def detectFace(self, userdata, img):
        # print userdata
        # rospy.loginfo("waiting...")
        try:
            self.frame = self.bridge.imgmsg_to_cv(img, "bgr8")
        except CvBridgeError, e:
            print e
            return 'invalid'

        self.frame = np.array(self.frame, dtype = np.uint8)
        face_box = self.process_image(self.frame)
        if face_box != None:
            x,y,w,h = face_box
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 255))
        cv2.waitKey(3)
        # self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        cv2.imshow(self.task, self.frame)
        # try:
        #     self.impub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
        return 'valid'
        
    def process_image(self,frame):
        gray =  cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        if self.cascade1:
            faces = self.cascade1.detectMultiScale(gray, **self.haar_params)

        if len(faces) == 0 and self.cascade3:
          faces = self.cascade3.detectMultiScale(gray, **self.haar_params)    

        if len(faces) == 0 and self.cascade2:
          faces = self.cascade2.detectMultiScale(gray, **self.haar_params)

        if len(faces) > 0:
            face_box = faces[0]
            # rospy.loginfo("face detected")
            print face_box
            return face_box
        else:
            # rospy.loginfo("no faces")
            return None

    def __init__(self):

        self.bridge = CvBridge()

        self.is_visible = False
        self.arList = []

        self.task = 'abcd'
        cv.NamedWindow(self.task, cv.CV_WINDOW_NORMAL)
        # cv.ResizeWindow(self.task, 480, 640)
        # cv2.imshow(self.task, np.zeros((480,640), dtype = np.uint8))
        rospy.loginfo("waiting for images...")

        cascade_1 = "/home/mscv/ros/hydro/catkin_ws/src/haar_detectors/haarcascade_frontalface_alt2.xml"
        self.cascade1 = cv2.CascadeClassifier(cascade_1)
        cascade_2 = "/home/mscv/ros/hydro/catkin_ws/src/haar_detectors/haarcascade_frontalface_alt.xml"
        self.cascade2 = cv2.CascadeClassifier(cascade_2)
        cascade_3 = "/home/mscv/ros/hydro/catkin_ws/src/haar_detectors/haarcascade_profileface.xml"
        self.cascade3 = cv2.CascadeClassifier(cascade_3)

        self.haar_params = dict(scaleFactor = 1.3,minNeighbors = 3,
                                flags = cv.CV_HAAR_DO_CANNY_PRUNING,
                                minSize = (25, 25),maxSize = (150, 150))

        rospy.init_node('find_treasure', anonymous=False)
        self.impub = rospy.Publisher('faceimg', Image)
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)

        ''' Create individual state machines for assigning tasks to each position '''

        # Initialize the overall state machine
        sm_find_treasure = StateMachine(outcomes=['succeeded','aborted','preempted'])
            
        # Build the find treasure state machine from the nav states and treasure finding states
        with sm_find_treasure:

            sm_nav = StateMachine(outcomes=['succeeded','aborted','preempted'])
            sm_nav.userdata.waypoints = self.waypoints
            sm_nav.userdata.waypoints_primary_count = len(self.waypoints)
            print sm_nav.userdata.waypoints

            with sm_nav:
                StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
                             transitions={'succeeded':'NAV_WAYPOINT','aborted':'','preempted':''},
                             remapping={'waypoint_out':'patrol_waypoint',
                                        'waypoints_primary_count':'waypoints_primary_count'})
            
                StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'PICK_WAYPOINT', 
                                          'aborted':'PICK_WAYPOINT', 
                                          'preempted':'PICK_WAYPOINT'},
                             remapping={'waypoint_in':'patrol_waypoint',
                                        'waypoints_primary_count':'waypoints_primary_count'})


            sm_read_tags = StateMachine(outcomes=['valid','invalid','preempted'])
            sm_read_tags.userdata.waypoints = self.waypoints

            with sm_read_tags:
                StateMachine.add('read_tag', ReadTagState('ar_pose_marker', AlvarMarkers)\
                    , transitions={'invalid':'read_tag','valid':'read_tag','preempted':''})


            sm_detect_faces = StateMachine(outcomes=['succeeded','aborted','preempted'])

            with sm_detect_faces:
                StateMachine.add('detect_face', MonitorState("/camera/rgb/image_color", Image, self.detectFace)
                , transitions={'invalid':'detect_face', 'valid':'detect_face', 'preempted':''})

            sm_con = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],default_outcome='succeeded',
            child_termination_cb=self.child_term_cb, outcome_cb=self.out_cb)

            with sm_con:
                Concurrence.add('SM_NAV', sm_nav)
                Concurrence.add('SM_READ_TAGS', sm_read_tags)
                Concurrence.add('SM_DETECT_FACES', sm_detect_faces)

            StateMachine.add('CON',sm_con, transitions={'succeeded':'','aborted':'','preempted':''})
                        
        # Create and start the SMACH introspection server sm_find_treasure
        intro_server = IntrospectionServer('find_treasure', sm_find_treasure, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = sm_find_treasure.execute()

        print('the length now is')
        print(self.waypoints)

        intro_server.stop()
            
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # sm_find_treasure.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Treasure finding test finished.")
