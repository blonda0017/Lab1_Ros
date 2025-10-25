#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"node2 received: {data.data}")

def node2():
    rospy.init_node('node2')
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    node2()
