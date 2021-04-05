#!/usr/bin/env python3
import rospy
from math import pi
from geometry_msgs.msg import Vector3, Twist

class DriveSquare(object):
    """ This node tells the robot to move in a square """
    
    def __init__(self):
        rospy.init_node('drive_square')
        self.drive_square_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.sleep(1) # give the node time to set up the publisher

        self.lin_speed = 0.2
        self.go_time = 5
        self.ang_speed = 0.3
        self.turn_time = pi/(2*self.ang_speed)
        self.v0 = Vector3(x=0.0,y=0.0,z=0.0)
        self.move = Twist(linear=self.v0, angular=self.v0)

    def run(self):
        cnt = 0

        # perform the instruction sequence 4 times
        while cnt<4:
            # move forward for 5 seconds
            self.move.linear = Vector3(x=self.lin_speed,y=0.0,z=0.0)
            self.move.angular = self.v0
            self.drive_square_pub.publish(self.move)
            rospy.sleep(self.go_time)

            # turn 90 degrees
            self.move.linear = self.v0
            self.move.angular = Vector3(x=0.0,y=0.0,z=self.ang_speed)
            self.drive_square_pub.publish(self.move)
            rospy.sleep(self.turn_time)
            cnt += 1

        # stop the robot
        self.move.angular = self.v0
        self.drive_square_pub.publish(self.move)

if __name__ == '__main__':
    node = DriveSquare()
    node.run()