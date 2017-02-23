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

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None
        self.binary_image = None
        self.image_info_window  = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        self.image_info_window = 255*np.ones((500,500,3))
        self.counter = 0
        self.center_x = self.center_y = 0;
        self.k = .03
        self.error = 0;



    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        if(self.counter > 5):
            self.counter = 0
            self.image_info_window = 255*np.ones((500,500,3))

            cv2.putText(self.image_info_window,
                        'Color (b=%d,g=%d,r=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]),
                        (5,50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0,0,0))
        else:
            self.counter += 1

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.cv_image = cv2.GaussianBlur(self.image  ,(5,5),0)
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (0,170,150), (15,255,255))
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']

        self.imagesum = np.sum(self.binary_image)
        self.imagesum /= (255.0*3)
        self.imagesum /= (640.0)
        self.imagesum *= 10


        cv2.circle(self.image, (int(self.center_x), int(self.center_y)), int(self.imagesum), (128, 255, 128), thickness=-1)

    def dribble(self):
        action = Twist()
        action.linear.x = 1
        self.error = 320 - self.center_x
        action.angular.z = self.error * self.k

        if(self.imagesum > 250):
            action.linear.x = 0
            action.angular.z = 0
        self.pub.publish(action)



    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.dribble()
            if not self.cv_image is None and not self.binary_image is None:
                cv2.imshow('video_window', self.image)

                # cv2.imshow('image_info', self.image_info_window)
                cv2.waitKey(5)

            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
