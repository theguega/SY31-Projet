#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int32, Float32

class DistNode:
    def __init__(self):
        rospy.init_node("us_driver")
        self.publisher = rospy.Publisher("/distance_us", Float32, queue_size=10)
        self.subscriber = rospy.Subscriber("/ultrasound", Int32, self.callback)

        self.alpha=5813.678278357846
        self.beta=18.654788247686156
        
        self.max_dist=275
    
        self.distances=[]

    def callback(self, msg_us):
        msg_dist = Float32()
        
        if msg_us.data>5500 :
            self.publisher.publish(-1)
            return
        
        if len(self.distances)>50:
            self.distances = self.distances[1:50]
        
        self.distances.append(msg_us.data)
        msg_dist.data=np.median(self.distances)
            
        msg_dist.data=(msg_dist.data-self.beta)/self.alpha

        self.publisher.publish(msg_dist.data)

if __name__ == "__main__":
    try:
        node = DistNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
