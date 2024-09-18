#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from time import time as system_time

from tf.transformations import quaternion_from_euler


def coordinates_to_message(x, y, O, t):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ] = quaternion_from_euler(0.0, 0.0, O)
    msg.header.stamp = t
    msg.header.frame_id = "odom"
    return msg


class Odom2PoseNode:
    def __init__(self):
        rospy.init_node("odom2pose")

        # Constants
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_SEPARATION = 0.160
        self.MAG_OFFSET = np.pi / 2.0 - 0.07

        # Variables
        self.x_odom, self.y_odom, self.O_odom = 0, 0, 0
        self.x_gyro, self.y_gyro, self.O_gyro = 0, 0, 0
        self.x_magn, self.y_magn, self.O_magn = 0, 0, 0
        self.x_mixed, self.y_mixed, self.O_mixed = 0, 0, 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.prev_gyro_t = 0
        self.v = 0
        self.w = 0
        self.x, self.y, self.O = 0, 0, None

        # Publishers
        self.pub_enco = rospy.Publisher("/pose_enco", PoseStamped, queue_size=10)
        self.pub_gyro = rospy.Publisher("/pose_gyro", PoseStamped, queue_size=10)
        self.pub_magn = rospy.Publisher("/pose_magn", PoseStamped, queue_size=10)
        self.pub_final = rospy.Publisher("/pose_final", PoseStamped, queue_size=10)
        # Subscribers
        self.sub_enco = rospy.Subscriber(
            "/sensor_state", SensorState, self.callback_enco
        )
        self.sub_magn = rospy.Subscriber(
            "/magnetic_field", MagneticField, self.callback_magn
        )
        self.sub_final = rospy.Subscriber("/imu", Imu, self.callback_final)
        self.output_enco = None
        self.output_magn = None
        self.prev_time_enco = None

    def callback_enco(self, sensor_state):
        t = sensor_state.header.stamp.to_sec()
        if not self.prev_time_enco:
            self.prev_time_enco = t
            return
        dt = t - self.prev_time_enco
        self.prev_time_enco = t

        # Compute the differential in encoder count
        d_left_encoder = sensor_state.left_encoder - self.prev_left_encoder
        d_right_encoder = sensor_state.right_encoder - self.prev_right_encoder
        if self.prev_left_encoder == 0:
            self.prev_left_encoder = sensor_state.left_encoder
            self.prev_right_encoder = sensor_state.right_encoder
            return
        self.prev_right_encoder = sensor_state.right_encoder
        self.prev_left_encoder = sensor_state.left_encoder

        # After investigation, the average frequency of the encoder is 28.7 Hz (our TP subject was tolding us 20 Hz, but it's not the case)
        v_right = (
            (2 * np.pi / self.ENCODER_RESOLUTION * d_right_encoder) / dt
        ) * self.WHEEL_RADIUS
        v_left = (
            (2 * np.pi / self.ENCODER_RESOLUTION * d_left_encoder) / dt
        ) * self.WHEEL_RADIUS

        self.v = (v_left + v_right) / 2
        self.w = (v_right - v_left) / (self.WHEEL_SEPARATION)

        self.x_odom = self.x_odom + self.v * np.cos(self.O_odom) * dt
        self.y_odom = self.y_odom + self.v * np.sin(self.O_odom) * dt
        self.O_odom = self.O_odom + self.w * dt

        msg = coordinates_to_message(
            self.x_odom, self.y_odom, self.O_odom, sensor_state.header.stamp
        )
        self.output_enco = msg
        self.pub_enco.publish(msg)

    def callback_magn(self, magnetic_field):
        if self.v == 0:
            return

        w = (
            np.arctan2(magnetic_field.magnetic_field.y, magnetic_field.magnetic_field.x)
            + self.MAG_OFFSET
        )

        if self.O == None:
            self.O = w

        self.x_magn = self.x_magn + self.v * np.cos(self.O_magn)
        self.y_magn = self.y_magn + self.v * np.sin(self.O_magn)
        self.O_magn = w

        msg = coordinates_to_message(
            self.x_magn, self.y_magn, self.O_magn, magnetic_field.header.stamp
        )
        self.output_magn = msg
        self.pub_magn.publish(msg)

    def callback_final(self, gyro):

        if not self.v or self.v == 0:
            return

        # Compute the elapsed time
        t = gyro.header.stamp.to_sec()
        dt = t - self.prev_gyro_t
        if self.prev_gyro_t == 0:
            self.prev_gyro_t = t
            return
        self.prev_gyro_t = t

        self.x_gyro += self.v * np.cos(self.O_gyro) * dt
        self.y_gyro += self.v * np.sin(self.O_gyro) * dt
        self.O_gyro = self.O_gyro + gyro.angular_velocity.z * dt

        msg_gyro = coordinates_to_message(
            self.x_gyro, self.y_gyro, self.O_gyro, gyro.header.stamp
        )
        self.pub_gyro.publish(msg_gyro)

        self.x_mixed = self.x_mixed + self.v * np.cos(self.O_mixed) * dt
        self.y_mixed = self.y_mixed + self.v * np.sin(self.O_mixed) * dt
        self.O_mixed = self.O_gyro

        msg_mixed = coordinates_to_message(
            self.x_mixed, self.y_mixed, self.O_mixed, gyro.header.stamp
        )
        self.pub_final.publish(msg_mixed)


if __name__ == "__main__":
    node = Odom2PoseNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
