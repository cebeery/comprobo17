#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import math

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        #set window names 
        cv2.namedWindow('video_window')
        cv2.namedWindow('threshold_image')
        
        #set callback for show color under mouse
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        #create color bounds and color bound gui controls
        self.green_lower_bound = 80
        self.red_upper_bound = 50
        self.blue_upper_bound = 50
        cv2.createTrackbar('green lower bound', 'threshold_image', self.green_lower_bound, 255, self.set_green_lower_bound)
        cv2.createTrackbar('red upper bound', 'threshold_image', self.red_upper_bound, 255, self.set_red_upper_bound)
        cv2.createTrackbar('blue upper bound', 'threshold_image', self.blue_upper_bound, 255, self.set_blue_upper_bound)

        #rosnode pubscribers
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        #control parameters
        self.kp = .005

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (0, self.green_lower_bound, 0), (self.blue_upper_bound,255, self.red_upper_bound))


    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def set_green_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.green_lower_bound = val

    def set_red_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_upper_bound = val

    def set_blue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.blue_upper_bound = val

    def set_twist(self):
        """ Setting the Twist velocity of the robot using proportional control. """

        window_x = 320
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            diff_x = self.center_x - window_x

          
            if math.fabs(diff_x) < 20:
                #move forward if within 20 pixels ahead
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0
            else:
                #rotate at a speed proportional to offest from center 
                self.twist.linear.x = 0
                self.twist.angular.z = -diff_x*self.kp

    def stop(self):
        """ This function publishes a twist to make the robot stop."""
        self.pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))       

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        rospy.on_shutdown(self.stop)

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                #show images in windows
                cv2.imshow('video_window', self.cv_image)
                cv2.imshow('threshold_image', self.binary_image)
                cv2.waitKey(5) 

                #move robot
                self.set_twist()
                self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
