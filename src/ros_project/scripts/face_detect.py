import actionlib
import cv2
import cv2.cv as cv
import datetime
import numpy as np 
import os
import rospy
import rospkg
import smach
import threading

from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from random import randrange
from sensor_msgs.msg import Image, CameraInfo
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from sound_utils import SoundUtils

class FaceDetectState(MonitorState):
    def __init__(self, topic, msg_type, max_checks = -1):
        State.__init__(self, outcomes=['invalid','valid','preempted'])        
        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = self.execute_cb
        self._max_checks = max_checks
        self._n_checks = 0

        self._trigger_cond = threading.Condition()

        self.bridge = CvBridge()
        rospy.sleep(1)
        self.task = 'Face Detection'
        cv.NamedWindow(self.task, cv.CV_WINDOW_NORMAL)
        # cv.ResizeWindow(self.task, 640, 480)
        # cv2.imshow(self.task, np.zeros((480,640), dtype = np.uint8))
        rospy.loginfo("waiting for images...")

        pkg_path = rospkg.RosPack().get_path('ros_project')

        cascade_1 = pkg_path+"/data/haar_detectors/haarcascade_frontalface_alt.xml"
        self.cascade1 = cv2.CascadeClassifier(cascade_1)
        cascade_2 = pkg_path+"/data/haar_detectors/haarcascade_frontalface_alt2.xml"
        self.cascade2 = cv2.CascadeClassifier(cascade_2)
        cascade_3 = pkg_path+"/data/haar_detectors/haarcascade_profileface.xml"
        self.cascade3 = cv2.CascadeClassifier(cascade_3)

        self.haar_params = dict(scaleFactor = 1.3,minNeighbors = 3,
                                flags = cv.CV_HAAR_DO_CANNY_PRUNING,
                                minSize = (30, 30),maxSize = (150, 150))

    def execute_cb(self, userdata, img):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
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
        self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        cv2.imshow(self.task, self.frame)
        return 'valid'
        
    def process_image(self,frame):
        gray = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        if self.cascade1:
            faces = self.cascade1.detectMultiScale(gray, **self.haar_params)

        if len(faces) == 0 and self.cascade2:
          faces = self.cascade2.detectMultiScale(gray, **self.haar_params)

        # if len(faces) == 0 and self.cascade3:
        #   faces = self.cascade3.detectMultiScale(gray, **self.haar_params)    

        if len(faces) > 0:
            face_box = faces[0]
            # print face_box
            # rospy.loginfo("face detected")
            SoundUtils.get_instance().say_message('Face Detected')
            return face_box
        else:
            # rospy.loginfo("no faces")
            return None
