#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class DriveSquare(object):
    '''
    This node moves the robot in a square.
    '''

    def __init__(self):
        # Define the node and open the subscriber and publisher
        rospy.init_node("drive_square")
        # rospy.Subscriber("/scan", LaserScan, self.readScan)
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.sleep(1)
      

    def run(self):

        # Offsets the transition time to meet the velocity
        offset = .0053 
        while not rospy.is_shutdown():
    
            # Move the robot forward for 4 seconds
            # Create a new velocity object
            new_vel = Twist()
            new_vel.linear.x = 0.1
            new_vel.angular.z = 0.0
            self.publisher.publish(new_vel)
            rospy.sleep(4)
           
            # Turn the robot 90 degrees over 4 seconds
            new_vel = Twist()
            # Angle difference for 4 seconds is 0.3927
            new_vel.angular.z = .3927 + offset
            new_vel.linear.x = 0.0
            self.publisher.publish(new_vel)
            rospy.sleep(4)

if __name__ == '__main__':
    node = DriveSquare()
    node.run()
