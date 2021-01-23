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
        self.notatwall = True
    
    # Process the output from the scan topic
    def processScan(self, data):
        # print("The nearest object: " + str(data.ranges[89]))
        # 1. send the robot forward toward the wall
        turnpoint = 1
        if data.ranges[0] > turnpoint and self.notatwall:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            
        # 2. turn at wall
        elif data.ranges[0] < turnpoint:
            self.twist.linear.x = 0.1
            error = (turnpoint - data.ranges[0])/turnpoint
            kp = 1
            velocity = kp * error
            self.twist.angular.z = velocity
            self.notatwall = False
        
        elif data.ranges[0] >= turnpoint and not self.notatwall:
            print("The nearest object: " + str(data.ranges[89]))
            self.twist.linear.x = 0.1
            error = (turnpoint - data.ranges[89])/turnpoint
            self.twist.angular.z = error
        self.pub.publish(self.twist)
        # 3. turn the robot about 90
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()

# class WallFollower(object):

#     def __init__(self):
#         # Initializes the rospy node
#         rospy.init_node("wall_follower")
#         # Connects to the /cmd_vel topic to update angular and linear velocity
#         self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#         # Lets the node rest for a moment to ensure the publisher is connected
#         rospy.sleep(1)
#         rospy.Subscriber("/scan", LaserScan, self.processScan)
#         self.twist = Twist()
#         self.hitwall = False
    
#     # Process the output from the scan topic
#     def processScan(self, data):
#         print("The nearest object: " + str(data.ranges[0]))
#         # 1. send the robot forward toward the wall
#         # error would be close
#         turnpoint = 1
#         if data.ranges[0] > turnpoint:
#             self.twist.linear.x = 0.1
#             self.twist.angular.z = 0.0

#         # 2. once the robot reaches a specific scan distance stop
#         elif data.ranges[0] < turnpoint:
#             self.twist.linear.x = 0.1
#             error = (turnpoint - data.ranges[0])/turnpoint
#             kp = 1
#             velocity = kp * error
#             self.twist.angular.z = velocity
#         self.pub.publish(self.twist)
#         # 3. turn the robot about 90
        

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     node = WallFollower()
#     node.run()