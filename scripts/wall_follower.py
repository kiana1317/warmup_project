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
        self.angle90 = None
        self.angle45 = None
        self.maxiDist = 0
        self.turning = True
        self.dist270 = 0
    
    # Process the output from the scan topic
    def processScan(self, data):
        turnpoint = 1
        if data.ranges[0] > turnpoint and self.notatwall:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            return
        
        # 2. once the robot reaches a specific scan distance stop
        if data.ranges[0] < turnpoint:
            self.twist.linear.x = 0.1
            error = (turnpoint - data.ranges[0])/turnpoint
            kp = 1
            velocity = kp * error
            self.twist.angular.z = velocity
            # if self.turning:
            self.dist270 = data.ranges[269] 
                # self.turning = False

        elif data.ranges[0] > turnpoint:
            self.twist.linear.x = 0.2
            kp = 1.1
            error = (self.dist270 - data.ranges[269])/ self.dist270 
            self.twist.angular.z = error * kp
            print("here")


        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
        # dists = data.ranges
        
        # turnpoint = 0.7
        # offset = .0053 
        # # Move the robot to a wall
        # if data.ranges[0] > turnpoint and self.notatwall:
        #     self.twist.linear.x = 0.2
        #     self.twist.angular.z = 0.0
        #     self.pub.publish(self.twist)
        #     return
        # # Acknowledge that the robot has hit the wall
        # elif self.notatwall:
        #     self.notatwall = False
        #     # directions ={
        #     #     'right': dists[224: 314],
        #     #     'left': dists[44: 134],
        #     #     'front': dists[314: ] + dists[:44],
        #     #     'back': dists[134 : 224],
        #     #     'front-left': dists[:44]
        #     # }
        #     # self.minDist = min(dists)
        #     # maxiAngle = dists.index(self.minDist)
        #     # print("The min angle: " + str(maxiAngle) + ". The max distance: " + str(self.minDist))
        
        # if data.ranges[0] < turnpoint and not self.turning:
        # # Turn the robot 90 degrees over 4 seconds
        #     # Angle difference for 4 seconds is 0.3927
        #     print(data.ranges[0])
        #     self.twist.angular.z = .3927 + offset
        #     self.twist.linear.x = 0.0
        #     self.pub.publish(self.twist)
        #     rospy.sleep(4)
        #     self.turning = True
    
        # if turning
        # self.twist.linear.x = 0.1
        # self.twist.angular.z = 0.0
        # self.pub.publish(self.twist)
        # Actions given the robot reached the wall
        
        # print("Front: " + str(directions['front']))
        # print("Back: " + str(directions['back']))
        # print("left: " + str(directions['left']))
        # print("right: " + str(directions['right']))
      
        


        # # print("The nearest object: " + str(data.ranges[89]))
        # # 1. send the robot forward toward the wall
        # turnpoint = 0.5
        # if data.ranges[0] > turnpoint and self.notatwall:
        #     self.twist.linear.x = 0.1
        #     self.twist.angular.z = 0.0

        # # 2. once the robot reaches a specific scan distance stop
        # else:
        #     # self.notatwall = False
        #     # Turn until there is no wall in the 45 degree pov of the robot
        #     # if not self.angle90:

        #     self.twist.linear.x = 0.0
        #     # error = (0.2 - data.ranges[0])/turnpoint
        #     # kp = 0.5
        #     # velocity = kp * error
        #     self.twist.angular.z = 0.5
        # self.pub.publish(self.twist)
        # 3. turn the robot about 90
        
        # elif data.ranges[0] <= turnpoint:

        
        # 2. turn at wall
        # elif data.ranges[0] <= turnpoint:
        #     if not self.angle90:
        #         self.angle90 = data.ranges[89]
            
            
        #     self.twist.linear.x = 0.0
        #     error = (turnpoint - data.ranges[0])/turnpoint
        #     kp = 0.5
        #     velocity = kp * error
        #     self.twist.angular.z = velocity
        #     self.notatwall = False
        
        # else:
        #     print("The nearest object: " + str(data.ranges[269]))
        #     self.twist.linear.x = 0.1
        #     # if data.ranges[269] == float("inf"):
        #     #     error = 1
        #     # else:
        #     # error = (turnpoint - data.ranges[269])/turnpoint
        #     kp = 0.5
        #     velocity = kp * error
        #     self.twist.angular.z = 0.0
        # 3. turn the robot about 90
        



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