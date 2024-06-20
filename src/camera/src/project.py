#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node("project")

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        # P, K and D are the camera parameters found by ROS calibration
        # (P: projection matrix, K: intrinsic parameters and D: distortion parameters, see CameraInfo for more details)
        self.P = self.K = self.D = None

        # Constants for the checkerboard detection and projection
        self.board_size = (8, 6)
        self.objp = np.zeros((np.prod(self.board_size), 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            : self.board_size[0], : self.board_size[1]
        ].T.reshape(-1, 2)

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher("~output", Image, queue_size=1)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber_info = rospy.Subscriber(
            "/turtlebotcam/camera_info", CameraInfo, self.callback_cam_info
        )
        self.subscriber_imag = rospy.Subscriber(
            "/camera/image_rect_color", Image, self.callback, queue_size=1
        )

    def callback_cam_info(self, msg):
        """
        Function called when a CameraInfo message is received.
        Retrieves the camera intrinsic parameters from the ROS calibration.
        Once the parameters are received, the topic is not listened to anymore
        msg: CameraInfo message received
        """
        self.P = np.array([msg.P[0:4], msg.P[4:8], msg.P[8:12]])
        self.K = np.resize(3, 3)
        self.D = np.resize(5, 1)

    def drawAxes(self, img, axis):
        """
        Function called when an image message is received.
        Retrieves the camera intrinsic parameters from the ROS calibration.
        Once the parameters are received, the topic is not listened to anymore
        img: OpenCV/numpy matrix
        axis: 2x4 matrix describing the coordinates of the origin and 3 axes in image space
        return: OpenCV/numpy matrix with lines drawn on it
        """
        axis = axis.astype(int)
        img = cv2.line(img, tuple(axis[:, 0]), tuple(axis[:, 1]), (255, 0, 0), 5)
        img = cv2.line(img, tuple(axis[:, 0]), tuple(axis[:, 2]), (0, 255, 0), 5)
        img = cv2.line(img, tuple(axis[:, 0]), tuple(axis[:, 3]), (0, 0, 255), 5)
        return img

    def callback(self, msg):
        """
        Function called when an image is received.
        msg: Image message received
        img: Width*Height*3 Numpy matrix storing the image
        """
        # Convert ROS Image -> OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # As we use the calibration information from ROS, die if it does not exist
        if self.P is None or self.P[0, 0] == 0:
            rospy.logwarn("Camera is not calibrated")
            return

        # u,v,w = np.matmul(self.P,np.array([[0.005],[0],[0.01],[1]]))
        # cv2.circle(img, (int(u/w),int(v/w)), 5, (0,0,255), -1)

        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        retval, corners = cv2.findChessboardCorners(img, self.board_size)

        cv2.cornerSubPix(
            img,
            corners,
            self.board_size,
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001),
        )
        retval, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
        print(rvec)
        print(tvec)
        print("------")

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn(e)


if __name__ == "__main__":
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
