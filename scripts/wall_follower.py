#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
from math import pi, sqrt

class WallFollower(object):
    # This tells the robot to drive alongside a wall at a distance

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
        min_dis = min(data.ranges)
        min_ang = data.ranges.index(min_dis)
        dis0 = data.ranges[0]
        dis90 = data.ranges[90]
        dis45 = data.ranges[45]
        front_min = min(data.ranges[0:35]+data.ranges[315:359])
        
        left_min = min(data.ranges[225:315])
        side_min = min(data.ranges[45:135])

        # first navigate to a wall
        if self.find_wall == 0:
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
            if dis0<0.4:
                self.twist.linear.x = 0
                self.twist.angular.z = 0.2
                self.find_wall = 1
            self.move_pub.publish(self.twist)
        else: 
            # if detects a wall in the front
            if front_min < 0.3*sqrt(2): #and dis0 < dis45:
                self.twist.angular.z = 0.2
                self.twist.linear.x = 0.0
            # maintain parallel
            else:
                self.twist.angular.z = 0.001*(side_min-90)
                self.twist.linear.x = 0.2
       
        """
        #  if the robot is not next to a wall
        if min_dis > 0.3:
            # move forward
            self.twist.angular.z = 0
            self.twist.linear.x = 0.2
        # if next to wall but not parallel, turn
        elif not (min_ang>45 and min_ang<135):
            # if the robot detects a wall in front of it, then turn
            if front_min < 0.3*sqrt(2):
                self.twist.angular.z = 0.1
                self.twist.linear.x = 0.0
            else: # turn such that 90 degree has minimum distance and move forward
                self.twist.angular.z = 0.02*(min_ang-90)
                self.twist.linear.x = 0.1
        
        """
        self.move_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()

