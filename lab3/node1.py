#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def node1():
    rospy.init_node('node1')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    
    while not rospy.is_shutdown():
        msg = "Hello from node1"
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        node1()
    except rospy.ROSInterruptException:
        pass
