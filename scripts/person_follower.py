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
        mindist = min(data.ranges)
        # Check if the robot is too close to the object
        if data.ranges[0] < 0.5:
            # Stop the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        # Check if there is an object in the room     
        elif mindist !=float("inf") :
            # Do not turn if the object is directly in front of the robot
            if data.ranges[0] == mindist:
                z = 0
            # Turn right if the object is between 180 to 360 degrees
            elif data.ranges.index(mindist) >= 180:
                z = -1
            # Turn left for an object located between 0 and 180 degrees
            else:
                z = 1
            # Set the velocities of the robot
            self.twist.linear.x = 0.5
            self.twist.angular.z = z


        # Publish the new angular and linear velocities 
        self.pub.publish(self.twist)

       
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("person_follower")
    node = PersonFollower()
    node.run()