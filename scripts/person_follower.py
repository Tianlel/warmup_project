#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(object):
    # this tells the robot to follow a person while maintaining a safe distance

    def __init__(self):
        rospy.init_node('follow_person')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        self.sensor = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.move = Twist(
            linear = Vector3(x=1,y=0,z=0),
            angular = Vector3(x=0,y=0,z=0)
        )

    def process_scan(self, data):
        # find angle corresponding to the shortest distance
        er_ang = data.ranges.index(min(data.ranges))

        # proportional angle control that instructs robot face the person
        if er_ang < 180:
            self.move.angular.z = er_ang*0.01
        else:
            self.move.angular.z = (er_ang-360)*0.01

        # proportional speed control that drives the robot towards the person
        er_lin = min(data.ranges)
        if er_lin>1:
            self.move.linear.x = 0.5
        else: 
            self.move.linear.x = (er_lin-0.3)*0.2

        self.vel_pub.publish(self.move)

    def run(self):
        rospy.spin()

if __name__=='__main__':
    node = PersonFollower()
    node.run()
