#!/usr/bin/env python

"""This script is our first CompRobo ROS node (publisher)"""

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

#iniitalize
rospy.init_node('test_message')
publisher = rospy.Publisher('/mypointtopic', PointStamped, queue_size=10)

#message
my_point_stamped = PointStamped(header=Header(stamp=rospy.Time.now(),
                                              frame_id="odom"),
                                point=Point(3.2,5.4,0.0))

#loop and update
rate = rospy.Rate(2)
while not rospy.is_shutdown(): 
    my_point_stamped.header.stamp = rospy.Time.now()
    publisher.publish(my_point_stamped)
    rate.sleep()

print "node is finshed"
