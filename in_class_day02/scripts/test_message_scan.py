#!/usr/bin/env python

"""Prints the laser scan range value for immediately fore of the Neato"""

import rospy
from sensor_msgs.msg import LaserScan

def process_scan(m):
    print m.ranges[0]

rospy.init_node('test_message_scan')
publisher = rospy.Subscriber('/scan', LaserScan, process_scan)

rate = rospy.Rate(2)
while not rospy.is_shutdown(): 
    rate.sleep()


print "node is finshed"
