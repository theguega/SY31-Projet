#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

class MappingNode:
    def __init__(self):
        rospy.init_node('mapping')

        # Publishers
        self.mapper = rospy.Publisher('/map', PointCloud2, queue_size=10)
        
        # Subscribers
        self.odom = rospy.Subscriber('/pose_enco', PoseStamped, self.callback_odom)
        self.lidar = rospy.Subscriber('/lidar/clusters', PointCloud2, self.callback_lidar)

    def callback_lidar(self, lidar):
        self.lidar = lidar
        return
    
    def callback_odom(self, odom):
        self.odom = odom
        return
    
    def callback_final(self):
        
        return
        
if __name__ == '__main__':
    node = MappingNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
