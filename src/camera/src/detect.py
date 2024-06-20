#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32


class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node("detect")

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.Blue = np.array([166, 114, 3])
        self.Bmin = self.Blue * 0.8
        self.Bmax = self.Blue * 1.2

        self.Red = np.array([123, 102, 187])
        self.Rmin = self.Red * 0.8
        self.Rmax = self.Red * 1.2

        self.dist = 0
        # Publisher to the output topics.
        self.pub_img_blue = rospy.Publisher("~output/blue", Image, queue_size=10)
        self.pub_img_red = rospy.Publisher("~output/red", Image, queue_size=10)

        self.pub_color = rospy.Publisher("~output/color", String, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.sub_bleu = rospy.Subscriber(
            "/camera/image_rect_color/compressed", CompressedImage, self.callback_blue
        )
        self.sub_red = rospy.Subscriber(
            "/camera/image_rect_color/compressed", CompressedImage, self.callback_red
        )
        self.sub_dist = rospy.Subscriber("/distance_us", Float32, self.callback_dist)

    def callback_dist(self, msg):
        self.dist = msg.data
        return

    def callback_blue(self, msg):
        """
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        """
        # Convert ROS CompressedImage -> OpenCV
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # height, width, _ = img_bgr.shape
            # img_bgr = img_bgr[height//4:3*height//4, width//4:3*width//4]
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Luv)
        mask = cv2.inRange(img_bgr, self.Bmin, self.Bmax)

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cv2.drawContours(img_bgr, c, -1, (0, 255, 0), 2)
            cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img_bgr, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255), -1)

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img_blue.publish(
                self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
            )  # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

        self.nb_c_blue = len(contours)

    def callback_red(self, msg):
        """
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        """
        # Convert ROS Image -> OpenCV
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # height, width, _ = img_bgr.shape
            # img_bgr = img_bgr[height//4:3*height//4, width//4:3*width//4]
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Luv)
        mask = cv2.inRange(img_bgr, self.Rmin, self.Rmax)

        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cv2.drawContours(img_bgr, c, -1, (0, 255, 0), 2)
            cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img_bgr, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255), -1)

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img_red.publish(
                self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
            )  # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

        self.nb_c_red = len(contours)

        if self.dist < 0.4:
            if self.nb_c_red > 400 or self.nb_c_blue > 400:
                if self.nb_c_red > self.nb_c_blue:
                    self.pub_color.publish("droite")
                else:
                    self.pub_color.publish("gauche")


if __name__ == "__main__":
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
