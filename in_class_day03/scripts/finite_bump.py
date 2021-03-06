#!/usr/bin/env python

"""Go forward until hit wall and then reverse"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan


class EmergencyStopNode(object):
    
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/bump', Bump, self.processBump) 
        rospy.Subscriber('/scan', LaserScan, self.processScan) 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5)
        self.cmd = Twist(linear = Vector3(x=0.5))

        self.states = {"F","B","L")
        self.current_state = "F"
        self.has_bumped = False
        self.has_seen = False
        self.timeTurnStart = rospy.Time.now()      
    
    def processBump(self, m):
        if m.leftFront or m.rightFront or m.leftFront or m.rightFront:
            self.has_bumped = True
            self.timeBumped = rospy.Time.now()
            print "Bumped"
        else:
            self.has_bumped = False 

    def processScan(self,m):
        if m.ranges[0] != 0.0:
            self.dist = m.ranges[0]
            if self.dist > 1.0:
                self.has_seen = True
                print "Wall Seen"
            else:
                self.has_seen = False
                print "Wall Gone"             

    def moveForward(self):
        """move foward until wall bump, then trigger backward state"""
        if self.has_bumped = True:
            self.current_state = "B"
            self.cmd.linear.x = -0.5
            self.cmd.angular.z = 0.0

    def moveBackward(self):
        """Move backward until laser distance from wall sufficient, then trigger turn state"""
        if self.wall_seen:
            self.has_bumped = True
            self.current_state = "L"
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.0
            self.timeTurnStart = rospy.Time.now()

    def turn(self):
        """continue turn until forward state starts"""
        if (rospy.Time.now() - self.timeTurnStart) > rospy.Duration(1.0):
            self.has_bumped = True
            self.current_state = "F"
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
            print "Timed Turn Complete"
            
       
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.cmd)
            self.rate.sleep()

        #end
        self.cmd = Twist()
        self.pub.publish(self.cmd)
        print 'Motion Ended'

   
scaredRobot = EmergencyStopNode()
scaredRobot.run()
