import actionlib
import cv2
import cv2.cv as cv
import datetime
import easygui
import numpy as np 
import rospy
import smach
import threading

from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, sin, pi
from nav_msgs.msg import Odometry
from pos import getPOSE
from smach import State
from tf.transformations import quaternion_from_euler
import numpy as np
import tf
##estimate pose from artag, only artag 7 has been calibrated to be aware!!!
class PoseEstimation(State):
    def __init__(self, topic, msg_type, max_checks = -1):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])  
        # self._topic = topic
        # self._msg_type = msg_type
        # self._cond_cb = self.doSomething
        # self._max_checks = max_checks
        # self._n_checks = 0

        # self._trigger_cond = threading.Condition()

            #3.3183; 0.43744; 0.36668
        # Turn the position locations into SMACH move_base action states
        self.tf_listener = tf.TransformListener()
        self.x = 0
        self.y = 0
        self.z = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.ow = 1
        self.odomReceived = False
        self.trans = 0
        self.rot = 0
        self.published = False
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = 1;
        self.init_pose.pose.pose.position.z = 0;
        self.init_pose.pose.pose.position.y = -1;

        self.init_pose.pose.pose.orientation.x = 0;
        self.init_pose.pose.pose.orientation.z = 0;
        self.init_pose.pose.pose.orientation.y = -0.707;
        self.init_pose.pose.pose.orientation.w = 0.707;

        self.init_pose.pose.covariance = [0.21236626335189102, 0.006923056032034009, 0.0, \
                0.0, 0.0, 0.0, 0.006923056032034009, 0.2105022764032361, 0.0, 0.0, \
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06977873322786521]

        self.initPose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=20)
        self.is_visible = False
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.doSomething)
        # self.poseOdom_sub = rospy.Subscriber('odom', Odometry, self.getPoseOdom)
        # while self.odomReceived is False:
            
        #     rospy.sleep(1)


    # def getPoseOdom(self,pmsg):
    #     self.x = pmsg.pose.pose.position.x
    #     self.y = pmsg.pose.pose.position.y
    #     self.z = pmsg.pose.pose.position.z

    #     self.ox = pmsg.pose.pose.orientation.x
    #     self.oy = pmsg.pose.pose.orientation.y
    #     self.oz = pmsg.pose.pose.orientation.z
    #     self.ow = pmsg.pose.pose.orientation.w

    #     self.odomReceived = True

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.published:
            rospy.sleep(4)
            return 'succeeded'

        return 'aborted'

    
    def get_odom(self):
        try:
            rospy.loginfo("Checking for transform between ar-tag and map frames")
            self.tf_listener.waitForTransform("/ar_marker_7", "/odom", rospy.Time(), rospy.Duration(4.0))
            rospy.loginfo("transform found :)")
            self.trans, self.rot = self.tf_listener.lookupTransform('/ar_marker_7',
'/odom', rospy.Time())
            ############self.rot is in quaternion############
            print "printing rotation"
            print self.rot
            rospy.loginfo("transformed successfully")
            return True
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF exception")
            return False
        

    def doSomething(self, msg):
        try:
            if self.published:
                return

            marker = msg.markers[0]
            if not self.is_visible:
                # rospy.loginfo("tags detected: %d", len(msg.markers))
                # rospy.loginfo("marker detected!!")

                self.is_visible = True;
                
                rospy.loginfo("ok let's check")
                if marker.id !=7:
                    rospy.loginfo("it's not tag 7")
                    return 'invalid'
                
                if not self.get_odom():
                    return 'invalid'

                TTT, RRR = getPOSE(np.asarray(self.trans), np.asarray(self.rot))
                self.init_pose.pose.pose.position.x = TTT.item(0)
                self.init_pose.pose.pose.position.y = TTT.item(1)
                self.init_pose.pose.pose.position.z = 0;

                self.init_pose.pose.pose.orientation.x = 0
                self.init_pose.pose.pose.orientation.y = 0
                self.init_pose.pose.pose.orientation.z = RRR[2]
                self.init_pose.pose.pose.orientation.w = RRR[3]
                self.init_pose.pose.covariance = [0.21236626335189102, 0.006923056032034009, 0.0, \
                0.0, 0.0, 0.0, 0.006923056032034009, 0.2105022764032361, 0.0, 0.0, \
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06977873322786521]

                rospy.sleep(0.1)
                rospy.loginfo("try  publishing pose")
                # print self.noPublished
                if not self.published:
                    rospy.loginfo("publishing pose")
                    self.initPose_pub.publish(self.init_pose)
                    rospy.loginfo("published")
                    self.published = True
                return 'valid'
        except:
            if self.is_visible:
                rospy.loginfo("marker not confirmed!!")
            self.is_visible = False
            return 'invalid'
     
        