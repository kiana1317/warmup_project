#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(object):
    
    def __init__(object):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PersonFollower()
    node.run()