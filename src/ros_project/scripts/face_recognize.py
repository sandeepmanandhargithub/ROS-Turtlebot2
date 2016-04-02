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
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import Image, CameraInfo
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from face_recognition.msg import FaceRecognitionAction, FaceRecognitionGoal, FaceRecognitionActionFeedback, FaceRecognitionActionResult
from sound_utils import SoundUtils

class FaceRecognitionState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                        input_keys=[],
                        output_keys=[])

        self._action_name = "face_recognition"
        
        self.face_recog_client = actionlib.SimpleActionClient(self._action_name, FaceRecognitionAction)
        
        # Wait up to 60 seconds for the action server to become available
        self.face_recog_client.wait_for_server(rospy.Duration(20))    
        
        rospy.loginfo("Connected to face_recognition action server")

        self._done_cond = threading.Condition() 

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self.goal = FaceRecognitionGoal()
        self.goal.order_id = 1
        self.goal.order_argument = ''

        self._done_cond.acquire() 
        self.face_recog_client.send_goal(self.goal, self._goal_done_cb, self._goal_active_cb, self._goal_feedback_cb)
 
        # Wait for action to finish 
        self._done_cond.wait() 
        self._done_cond.release()

        print self._goal_status
        print self._goal_result

        # self.goal.order_id = 4
        # self.face_recog_client.send_goal(self.goal)
        
        if self._goal_status == GoalStatus.PREEMPTED:
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def request_preempt(self): 
        smach.State.request_preempt(self) 
        self.face_recog_client.cancel_goal()

    def _goal_active_cb(self): 
        rospy.logdebug("Action "+self._action_name+" has gone active.") 

    def _goal_feedback_cb(self, feedback): 
        """Goal Feedback Callback""" 
        if(len(feedback.names) > 0):        
            SoundUtils.get_instance().say_message(feedback.names[0] + " is recognized")


    def _goal_done_cb(self, result_state, result): 
        self._goal_status = result_state 
        self._goal_result = result 
 
        self._done_cond.acquire() 
        self._done_cond.notify() 
        self._done_cond.release() 

