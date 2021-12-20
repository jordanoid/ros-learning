#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from math import sqrt
from location_monitor.msg import LandmarkDistance

class LandmarkMonitor():
    def __init__(self, pub, landmarks):
        self._pub = pub
        self._landmarks = landmarks

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        closest_distance = None
        for l_name, (l_x, l_y) in self._landmarks.items():
            dist = distance(x, y, l_x, l_y)
            if closest_distance is None or dist < closest_distance:
                closest_name = l_name
                closest_distance = dist
        ld = LandmarkDistance()
        ld.name = closest_name
        ld.distance = closest_distance
        self._pub.publish(ld)

def distance(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    return sqrt(xd**2 + yd**2)

def main():
    landmarks = {"Mailbox":(0.8834, -0.5762),
             "Table":(-2.6572, 2.4237),
             "Trash Can":(-4.6935, 4.8943),
             "Bookshelf":(-6.5439, 5.1948)}
    rospy.init_node("location_monitor")

    pub = rospy.Publisher("closest_landmark", LandmarkDistance, queue_size=10)
    monitor = LandmarkMonitor(pub, landmarks)

    rospy.Subscriber("/odom", Odometry, monitor.callback)
    rospy.spin()

if __name__ == "__main__":
    main()