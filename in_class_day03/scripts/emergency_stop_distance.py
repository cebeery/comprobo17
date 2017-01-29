#!/usr/bin/env python

"""Go forward until hit wall and then reverse"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class EmergencyStopNode(object):
    
    def __init__(self):
        rospy.init_node('emergency_stop_distance')
        rospy.Subscriber('/scan', LaserScan, self.processScan) 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5)
        self.cmd = Twist(linear = Vector3(x=0.5))
        self.has_seen = False
        self.timeBumped = rospy.Time.now()
        self.dist = 5.0

    def processScan(self,m):
        if m.ranges[0] != 0.0:
            self.dist = m.ranges[0]
            if self.dist < 1.0:
                self.has_seen = True
                self.cmd.linear.x = -0.5
                print "Wall Seen"
       

    def run(self):
        while not rospy.is_shutdown() and not (self.has_seen and self.dist > 3.0):
            self.pub.publish(self.cmd)
            self.rate.sleep()

        #end
        self.cmd = Twist()
        self.pub.publish(self.cmd)
        print 'Motion Ended'

   
scaredRobot = EmergencyStopNode()
scaredRobot.run()
