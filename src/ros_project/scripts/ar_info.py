#!/usr/bin/env python

""" cv_bridge_demo.py - Version 1.1 2013-12-20

  Trying to show the marker position by stamping it on image
  Try to get it on workstation first then stamp it

  Subscribe to /camera/rgb/image_color
  Subscribe to ar_pose_marker
      
"""

import rospy
import sys
import cv2
import cv2.cv as cv
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan2, asin, pi, sqrt
from ar_track_alvar.msg import AlvarMarkers


class ArInfo():
    def __init__(self, x, y, z, q1, q2, q3, q0, id):
        self.pose.x = x
        self.pose.y = y
        self.pose.z = z

        self.orientation.x = q1
        self.orientation.y = q2
        self.orientation.z = q3
        self.orientation.w = q0

        self.id = id




class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)
        
        # And one for the depth image

        
        self.target_visible = False
        self.tl = tf.TransformListener()

        self.xt = -1
        self.yt = -1
        self.w = -1
        self.i = -1
        self.j = -1
        self.k = -1
        self.yaw = -1;
        self.roll= -1;
        self.pitch = -1;
        self.MarkId = -1;
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
       
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.doSomething)

        self.is_visible = False;

        rospy.loginfo("Waiting for image topics...")


    def doSomething(self, msg):
    	"""
        		Try
      	"""
        try:

            rospy.loginfo("trying")
            rospy.loginfo("tags detected: %d", len(msg.markers))

            for tag in (msg.markers):
            	rospy.loginfo("%f, %f, %f",tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z);
            	
            marker = msg.markers[0]
            #rospy.loginfo(marker.id)
            if not self.is_visible:
                rospy.loginfo("marker detected!!")
                self.is_visible = True;
                
                tX = marker.pose.pose.position.x
                tY = marker.pose.pose.position.y

                if sqrt((self.xt - tX)**2 + (self.yt - tY)**2) < 0.05:
                	rospy.loginfo("distance from prev: %f", sqrt((self.xt - tX)**2 + (self.yt - tY)**2))
                	#preventing from relocalising the target position
                	return
                	
                self.xt = marker.pose.pose.position.x
                self.yt = marker.pose.pose.position.y

                w = marker.pose.pose.orientation.w
                i = marker.pose.pose.orientation.x
                j = marker.pose.pose.orientation.y
                k = marker.pose.pose.orientation.z
                self.yaw = atan2(2*j*w - 2*i*k, 1 - 2*j*j - 2*k*k)*180/pi
                self.pitch = asin(2*i*j + 2*k*w)*180/pi
                self.roll = atan2(2*i*w - 2*j*k, 1 - 2*i*i - 2*k*k)*180/pi
                rospy.loginfo("position @ (%f, %f)", self.xt, self.yt)
                rospy.loginfo("yaw, pitch, roll @ (%f, %f, %f)", self.yaw, self.pitch, self.roll)

                if self.yaw < 110 and self.yaw > 70 and self.roll > -10 and self.roll < 10:
                	dx = 0.5; dy = self.yt; 
                	# q = Quaternion(0, 0, 0.7071, 0.7071)
                	rospy.loginfo("!!tag faceing along x-axis!!")
                elif self.yaw < -70 and self.yaw > -110 and self.roll > -10 and self.roll < 10:
                	dx = -0.5; dy = self.yt; 
                	# q = Quaternion(0, 0, 0.7071, 0.7071)
                	rospy.loginfo("!!tag faceing against x-axis!!")
                elif self.yaw < 110 and self.yaw > 70 and self.roll < -70 and self.roll > -110:
                	dx = self.xt; dy = 0.5; 
                	# q = Quaternion(0, 0, 0.7071, 0.7071)
                	rospy.loginfo("!!tag faceing along y-axis!!")
                elif self.yaw < 110 and self.yaw > 70 and self.roll < 110 and self.roll > 70:
                	dx = self.xt; dy = -0.5; 
                	# q = Quaternion(0, 0, 0.7071, 0.7071)
                	rospy.loginfo("!!tag faceing against y-axis!!")

                self.MarkId = marker.id	

                if marker.id == 3:
                    """
                        try rotating according to the tag
                    """
                    
            
            
        except:
            if self.is_visible:
                rospy.loginfo("marker not confirmed!!")
            self.is_visible = False

        return
   			



        

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        frame = self.process_image(frame)
        font_face = cv2. FONT_HERSHEY_PLAIN
        font_scale = 1
        display_image = frame.copy()
        cv2.putText(display_image, "Marker@:" , (10,20),font_face, font_scale, cv.RGB(0, 0, 255) )    
        cv2.putText(display_image, "x: " + str(self.xt) , (10,40),font_face, font_scale, cv.RGB(0, 255, 0) )    
        cv2.putText(display_image, "y: " + str(self.yt), (10,54),font_face, font_scale, cv.RGB(0, 255, 0) )    
        cv2.putText(display_image, "Ort: " + str(self.yaw) + ", " + str(self.pitch) + ", " + str(self.roll), (10,68),font_face, font_scale, cv.RGB(0, 255, 0) )    
        cv2.putText(display_image, "iD: " + str(self.MarkId), (10,82),font_face, font_scale, cv.RGB(0, 255, 0) )    
           
        # cv2.putText(display_image, "FPS: " + str(self.fps), (10, 20), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, "Keyboard commands:", (20, int(self.frame_size[0] * 0.6)), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, " ", (20, int(self.Markx * 0.65)), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, "space - toggle pause/play", (20, int(self.frame_size[0] * 0.72)), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, "     r - restart video from beginning", (20, int(self.frame_size[0] * 0.79)), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, "     t - hide/show this text", (20, int(self.frame_size[0] * 0.86)), font_face, font_scale, cv.RGB(255, 255, 0))
        # cv2.putText(display_image, "     q - quit the program", (20, int(self.frame_size[0] * 0.93)), font_face, font_scale, cv.RGB(255, 255, 0))
            
            # Merge the original image and the display image (text overlay)
        display_image = cv2.bitwise_or(frame, display_image)

                       
        # Display the image.
        cv2.imshow(self.node_name, display_image)
        
        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
                
   
          
    def process_image(self, frame):
        # Convert to greyscale
        # grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
        # frame = cv2.blur(frame, (3, 3))
        
        # Compute edges using the Canny edge filter
        # edges = cv2.Canny(grey, 15.0, 30.0)
        
        return frame
    
 
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvBridgeDemo()

        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    