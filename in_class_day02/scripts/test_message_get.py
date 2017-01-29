#!/usr/bin/env python

"""This script is our second CompRobo ROS node (subscriber)"""

import rospy
from geometry_msgs.msg import PointStamped

def process_point(m):
    print m

rospy.init_node('test_message_get')
publisher = rospy.Subscriber('/mypointtopic', PointStamped, process_point)

rate = rospy.Rate(2)
while not rospy.is_shutdown(): 
    rate.sleep()


print "node is finshed"
