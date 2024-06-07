#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud, read_points
from tf.transformations import euler_from_quaternion

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]



class MappingNode:
    def __init__(self):
        rospy.init_node('mapping')
        # Map
        self.map = np.array([])

        # Publishers
        self.mapper = rospy.Publisher('/map', PointCloud2, queue_size=10)
        
        # Subscribers
        self.odom = rospy.Subscriber('/pose_enco', PoseStamped, self.callback_odom)
        self.lidar = rospy.Subscriber('/lidar/clusters', PointCloud2, self.callback_final)
    
    def callback_odom(self, odom):
        self.coords_robot = odom
        return
    
    def callback_final(self, msg):
        #robots position [x, y, theta]
        x_robot = self.coords_robot.pose.position.x
        y_robot = self.coords_robot.pose.position.y
        theta_robot = euler_from_quaternion([self.coords_robot.pose.orientation.x, self.coords_robot.pose.orientation.y, self.coords_robot.pose.orientation.z, self.coords_robot.pose.orientation.w])[2]

        # Transformation matrix
        T = np.array([[np.cos(theta_robot), -np.sin(theta_robot), x_robot],
                      [np.sin(theta_robot), np.cos(theta_robot), y_robot],
                      [0, 0, 1]])
        
        # Transform the points from the clusters
        points = np.array(list(read_points(msg)))[:,:2]
        points = np.hstack((points, np.ones((points.shape[0],1)))).transpose()
        points = np.dot(T, points)[:2].transpose()
        map_msg = create_cloud(msg.header, PC2FIELDS, [[points[i,0],points[i,1],0,0] for i in range(points.shape[0])])
        map_msg.header.frame_id = "base_scan"

        self.mapper.publish(map_msg)
        return
        
if __name__ == '__main__':
    node = MappingNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
