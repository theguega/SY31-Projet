#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def norm(p1, p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(1/2)

def callback(msg):
    clusters = {}
    bboxes = MarkerArray()
    for x,y,z,c in read_points(msg):
        if c not in clusters.keys(): clusters[c] = []
        clusters[c].append([x,y])

    for c, points in clusters.items():
        points = np.array(points)

        # ToDo: Calculate cluster center
        min = np.min(points, axis=0)
        max = np.max(points, axis=0)
        center = (max+min)/2

        # ToDo: Calculate cluster length and width
        dist = []
        for i in range(len(points)):
            dist.append(norm(center, points[i]))
        jmax = np.argmax(dist)
        
        width, length = [2*abs(center[0]-points[jmax][0]), 2*abs(center[1]-points[jmax][1])]

        if length<1 and width<1:
            bbox = Marker()
            bbox.header = msg.header
            bbox.id = c
            bbox.type = Marker.CUBE
            bbox.action = Marker.ADD
            bbox.pose.position = Point(center[0], center[1], 0)
            bbox.pose.orientation.w = 1
            bbox.scale.x, bbox.scale.y, bbox.scale.z = width, length, 0.3
            bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0, 0, 0.5
            bbox.lifetime = rospy.Duration(0.2)
            bboxes.markers.append(bbox)

    pub_bboxes.publish(bboxes)

if __name__ == '__main__':
    rospy.init_node('shaper_bbox')
    pub_bboxes = rospy.Publisher('/lidar/bboxes', MarkerArray, queue_size=10)
    rospy.Subscriber('/lidar/clusters', PointCloud2, callback)
    rospy.spin()
