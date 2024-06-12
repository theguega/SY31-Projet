#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def norm(p1, p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(1/2)

def callback(msg):
    points = np.array(list(read_points(msg)))[:,:2]
    groups = np.zeros(points.shape[0], dtype=int)

    
    k = 2
    # Clustering algorithm considering k-nearest neighbors
    for i in range(k, points.shape[0]):
        D =  0.005
         
        d = np.zeros(k-1)
        for j in range(1, k):
            d[j-1] = norm(points[i],points[i-j])
        dmin = min(d)
        jmin = np.argmin(d) + 1
        
        if dmin < D:
            if groups[i-jmin] == 0:
                groups[i-jmin]=max(groups)+1
            groups[i]=groups[i-jmin]
            
    # remove points of cluster 0
    non_zero_indices = np.where(groups != 0)[0]
    points = points[non_zero_indices]
    groups = groups[non_zero_indices]
        
    #remove cluster with small amout of points
    unique_groups, counts = np.unique(groups, return_counts=True)
    non_valid_groups = unique_groups[counts < 5]

    valid_mask = np.isin(groups, non_valid_groups, invert=True)
    points = points[valid_mask]
    groups = groups[valid_mask]
            

    clust_msg = create_cloud(msg.header, PC2FIELDS, [[points[i,0],points[i,1],0,c] for i,c in enumerate(groups)])
    pub_clusters.publish(clust_msg)

if __name__ == '__main__':
    rospy.init_node('clusterer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    rospy.Subscriber('/lidar/points', PointCloud2, callback)
    rospy.spin()
