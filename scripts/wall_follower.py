#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower(object):

    def __init__(self):
        # Initializes the rospy node
        rospy.init_node("wall_follower")
        # Connects to the /cmd_vel topic to update angular and linear velocity
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.processScan)
        # Creats a twist object
        self.twist = Twist()
        # A boolean to determine if the robot is at the wall
        self.notatwall = True
        
    # Process the output from the scan topic
    def processScan(self, data):
        # The distance from the wall that the robot starts turning
        turnpoint = 1.0

        # Part 1: Find the wall

        # Move the robot forward to find a wall
        if data.ranges[0] > turnpoint and self.notatwall:
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            return
        
        # Acknowledge that the robot has hit the wall
        elif self.notatwall:
            self.notatwall = False
        ############
        # Part 2: Follow the wall
        
        # The distance from the wall that the robot moves
        walldist = 0.2
        # Turn the robot counter clockwise if it reaches the turning point
        if data.ranges[0] <= turnpoint:
            self.twist.linear.x = 0.2
            # Proportional Control for the angular rotation
            error = (turnpoint - data.ranges[0])/turnpoint
            kp = 1.0
            velocity = kp * error
            self.twist.angular.z = velocity 
        # Move the robot along the wall
        else:
            self.twist.linear.x = 0.3
            # Adjust the robot's angular velocity to keep the robot parallel
            # to the wall
            error = (walldist - data.ranges[269])/walldist
            kp = 0.3
            velocity = kp * error
            self.twist.angular.z = velocity

        # Publish the twist 
        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
        