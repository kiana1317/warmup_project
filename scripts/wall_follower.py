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
        # Lets the node rest for a moment to ensure the publisher is connected
        rospy.sleep(1)
        rospy.Subscriber("/scan", LaserScan, self.processScan)
        self.twist = Twist()
    
    # Process the output from the scan topic
    def processScan(self, data):
        # 1. send the robot forward toward the wall
        if data.ranges[0] < 0.3:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.398
            self.pub.publish(self.twist)
            rospy.sleep(4)
        # Move forward if not near the wall
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
        # 2. once the robot reaches a specific scan distance stop
        # 3. turn the robot about 90
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()