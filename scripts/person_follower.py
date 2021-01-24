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
            # Turn the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.4
        # Check if the robot is too close to the object
        elif data.ranges[0] < 0.5:
            # Stop the robot
            print("Is digit is true with the value: " + str(data.ranges[0]))
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        
        else:
            # Move the robot forward
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
            
        self.pub.publish(self.twist)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("person_follower")
    node = PersonFollower()
    node.run()