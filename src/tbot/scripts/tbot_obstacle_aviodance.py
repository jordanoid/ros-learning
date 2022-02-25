#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Robot():
    def __init__(self, pub):
        self._pub = pub
    def scan_callback(self, msg):

        vel_msg = Twist()
        rate = rospy.Rate(10)
        
        def rotate():
            min_range = 3.0
            if(msg.ranges[-5] < min_range and msg.ranges[0] < min_range and msg.ranges[5] < min_range):
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.7
                self._pub.publish(vel_msg)
        def move():
            min_range = 0.6
            if(msg.ranges[-5] > min_range and msg.ranges[0] > min_range and msg.ranges[5] > min_range):
                vel_msg.linear.x = 0.3
                vel_msg.angular.z = 0.0
                self._pub.publish(vel_msg)
            else:
                rotate()

        move()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_aviodance')
    rate = rospy.Rate(1)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    robot1 = Robot(pub)
    rospy.Subscriber('/scan', LaserScan,robot1.scan_callback)
    rate.sleep()
    rospy.spin()

    
        

    
