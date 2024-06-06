#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.Cmin = np.array([190,140,0])
        self.Cmax= np.array([250,170,5])

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_rect_color', Image, self.callback)

    def callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return
        
        cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Luv)
        mask = cv2.inRange(img_bgr, self.Cmin, self.Cmax)
        
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)!=0:
            biggest_contour=max(contours, key=cv2.contourArea)
            x,y,w,h=cv2.boundingRect(biggest_contour)
            cv2.drawContours(img_bgr,biggest_contour,-1,(0,255,0),2)
            cv2.rectangle(img_bgr, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(img_bgr, (int(x+w/2),int(y+h/2)), 5, (0,0,255), -1)
        
        
        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
