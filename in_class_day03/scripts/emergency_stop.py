#!/usr/bin/env python

"""Go forward until hit wall and then reverse"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

class EmergencyStopNode(object):
    
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/bump', Bump, self.processBump) 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5)
        self.cmd = Twist(linear = Vector3(x=0.5))
        self.has_bumped = False
        self.timeBumped = rospy.Time.now()
       

    def processBump(self, m):
        if m.leftFront or m.rightFront:
            self.has_bumped = True
            self.cmd.linear.x = -0.5
            self.timeBumped = rospy.Time.now()
            print "Bumped"
       

    def run(self):
        while not rospy.is_shutdown() and not (self.has_bumped and (rospy.Time.now() - self.timeBumped) > rospy.Duration(2.0)):
            print self.cmd
            self.pub.publish(self.cmd)
            self.rate.sleep()

        #end
        self.cmd = Twist()
        self.pub.publish(self.cmd)
        print 'Motion Ended'

   
scaredRobot = EmergencyStopNode()
scaredRobot.run()
