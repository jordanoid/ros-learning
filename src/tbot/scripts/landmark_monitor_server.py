#!/usr/bin/env python3

from math import sqrt
import rospy
from tbot_msgs.srv import *
from nav_msgs.msg import Odometry

class LandmarkMonitor():
    def __init__(self):
        self._landmarks = {"Mailbox":(0.8834, -0.5762),
                           "Table":(-2.6572, 2.4237),
                           "Trash Can":(-4.6935, 4.8943),
                           "Bookshelf":(-6.5439, 5.1948)}
        self._x = 0
        self._y = 0

    def get_closest(self, req):
        rospy.loginfo("GetClosest  called")
        best_landmark = ""
        best_distance = -1
        for name, (x, y) in self._landmarks.items():
            dx = x - self._x
            dy = y - self._y
            sq_dist = dx**2 + dy**2
            if best_distance == -1 or sq_dist < best_distance:
                best_distance = sq_dist
                best_landmark = name
        
        response = GetClosestResponse()
        response.name = best_landmark
        return response

    def get_distance(self, req):
        rospy.loginfo(f"GetDistance called with {req.name}")
        if req.name not in self._landmarks:
            rospy.logerr(f"Unknown landmark {req.name}")
            return None
        x, y = self._landmarks[req.name]
        dx = x - self._x
        dy = y - self._y
        response = GetDistanceResponse()
        response.distance = sqrt(dx**2 + dy**2)
        return response

    def odom_callback(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y


def main():
    rospy.init_node("landmark_server")
    monitor = LandmarkMonitor()
    get_closest = rospy.Service("get_closest", GetClosest, monitor.get_closest)
    get_distance = rospy.Service("get_distance", GetDistance, monitor.get_distance)
    sub = rospy.Subscriber("/odom", Odometry, monitor.odom_callback)
    rospy.spin()

if __name__ == "__main__":
    main()