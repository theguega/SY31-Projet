#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud, read_points

class MappingNode:
    def __init__(self):
        rospy.init_node('mapping')

        # Publishers
        self.mapper = rospy.Publisher('/map', PointCloud2, queue_size=10)
        
        # Subscribers
        self.odom = rospy.Subscriber('/pose_enco', PoseStamped, self.callback_odom)
        self.lidar = rospy.Subscriber('/lidar/clusters', PointCloud2, self.callback_final)
    
    def callback_odom(self, odom):
        self.odom = odom
        return
    
    def callback_final(self, lidar):
        # Compute the transformation matrix to map the maze to PointCloud2
        theta_robot = self.odom.pose.orientation.w
        print(theta_robot)
        x_robot = self.odom.pose.position.x
        y_robot = self.odom.pose.position.y

        # Transformation matrix
        T = np.array([[np.cos(theta_robot), -np.sin(theta_robot), x_robot],
                      [np.sin(theta_robot), np.cos(theta_robot), y_robot],
                      [0, 0, 1]])
        
        # Get lidar points to numpy array
        # [x1, x2, ...;
        # y1, y2, ...;
        # 1,  1,  ...]
        points = np.array(list(read_points(lidar)))[:,:2].transpose()
        points = np.vstack((points, np.ones(points.shape[1])))
        
        # Apply transformation matrix
        points = np.dot(T, points)

        # Publish the transformed points
        map_msg = create_cloud(lidar.header, [PointField('x', 0, PointField.FLOAT32, 1),
                                              PointField('y', 4, PointField.FLOAT32, 1),
                                              PointField('z', 8, PointField.FLOAT32, 1),
                                              PointField('c', 12, PointField.INT16, 1)],
                                              [[points[0,i], points[1,i], 0, 0] for i in range(points.shape[1])])
        self.mapper.publish(map_msg)
        return
        
if __name__ == '__main__':
    node = MappingNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
