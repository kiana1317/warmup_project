#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(object):
    
    def __init__(self):
        # Connecting to the publisher for the robot's velocity
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Subscribing to the outputs from the scan topic
        rospy.Subscriber("/scan", LaserScan, self.processScan)
        # Initiating a twist object
        self.twist = Twist()

    def processScan(self, data):
        
        # Check if nothing is in front of the robot
        if data.ranges[0] == float("inf"):
            # The angular z velocity 
            z = 0.5

            # Rotate the robot clockwise if the object is to the robot's right
            for i in  range(180,360):
                if data.ranges[i] != float("inf"):
                    z = -1.0 * z 
                    break
            
            # Turn the robot in place
            self.twist.linear.x = 0.0
            self.twist.angular.z = z

        # Check if the robot is too close to the object
        elif data.ranges[0] < 0.5:
            # Stop the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        
        else:
            # Move the robot forward if there is an object in its sight
            self.twist.linear.x = 0.4
            self.twist.angular.z = 0.0

        # Publish the new angular and linear velocities 
        self.pub.publish(self.twist)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("person_follower")
    node = PersonFollower()
    node.run()