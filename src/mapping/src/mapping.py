#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from sensor_msgs.point_cloud2 import create_cloud, read_points
from tf.transformations import euler_from_quaternion

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def norm(p1, p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(1/2)


class MappingNode:
    def __init__(self):
        rospy.init_node('mapping')
        # Map
        self.map = np.array([])
        
        

        # Publishers
        self.mapper = rospy.Publisher('/map', PointCloud2, queue_size=10)
        
        # Subscribers
        self.odom = rospy.Subscriber('/pose_final', PoseStamped, self.callback_odom)
        self.lidar = rospy.Subscriber('/lidar/clusters', PointCloud2, self.callback_final)

    def callback_odom(self, odom):
        self.x_robot = odom.pose.position.x
        self.y_robot = odom.pose.position.y
        self.theta_robot = euler_from_quaternion([odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z, odom.pose.orientation.w])[2]
        return
    
    def callback_vitesse(self, msg):
        self.angle = msg.data
        return
    
    def callback_final(self, msg):    
        # Transformation matrix
        T = np.array([[np.cos(self.theta_robot), -np.sin(self.theta_robot), self.x_robot],
                      [np.sin(self.theta_robot), np.cos(self.theta_robot), self.y_robot],
                      [0, 0, 1]])
        
        # Transform the points from the clusters
        points = np.array(list(read_points(msg)))[:,:2]
        points = np.hstack((points, np.ones((points.shape[0],1)))).transpose()
        points = np.dot(T, points)[:2].transpose()

        # For debugging purposes
        points_msg = create_cloud(msg.header, PC2FIELDS, [[points[i,0],points[i,1],0,0] for i in range(points.shape[0])])

        # Add points to the map
        self.map = np.vstack((self.map, points)) if self.map.size else points
        
        map_msg = create_cloud(msg.header, PC2FIELDS, [[self.map[i,0],self.map[i,1],0,0] for i in range(self.map.shape[0])])
        map_msg.header.frame_id = "base_scan"

        self.mapper.publish(map_msg)
        return
        
if __name__ == '__main__':
    node = MappingNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
