#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
from math import pi, sqrt

class WallFollower(object):
    # This tells the robot to drive alongside a wall at a distance (counter-clockwise)

    def __init__(self):
        rospy.init_node('wall_follower')
        rospy.sleep(1)

        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sense = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        self.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.find_wall = 0

    def process_scan(self, data):
        dis0 = data.ranges[0]
        front_min = min(data.ranges[0:35]+data.ranges[315:359]) 
        right_min = min(data.ranges[45:135])

        # first drive the robot forward up to a wall
        if self.find_wall == 0:
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
            # if the robot is in front of the wall, stop and turn left
            if dis0<0.4:
                self.twist.linear.x = 0
                self.twist.angular.z = 0.2
                self.find_wall = 1
            self.move_pub.publish(self.twist)
        else: 
            # if detects a wall in the front (at a corner), stop and turn left
            if front_min < 0.3*sqrt(2): 
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.2
            # drive along wall while maintaining a distance
            else:
                self.twist.angular.z = 0.0005*(side_min-90)
                self.twist.linear.x = 0.2
    
        self.move_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()

