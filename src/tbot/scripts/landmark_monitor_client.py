#!/usr/bin/env python3

import rospy
from tbot_msgs.srv import *

def main():
    rospy.init_node("landmark_client")
    get_closest = rospy.ServiceProxy("get_closest", GetClosest)
    get_distance = rospy.ServiceProxy("get_distance", GetDistance)

    try:
        rospy.wait_for_service("get_closest", timeout=10)
        rospy.wait_for_service("get_distance", timeout=10)
    except rospy.ROSException:
        rospy.logerr("Service did not come up within 10 seconds")
        return

    req = GetClosestRequest()
    resp = get_closest(req)
    print(f"Closest: {resp.name}")

    landmarks =["Mailbox", "Table", "Trash Can", "Bookshelf"]
    for landmark in landmarks:
        req = GetDistanceRequest()
        req.name = landmark
        resp = get_distance(req)
        print(f"{landmark}: {resp.distance}")

if __name__ == '__main__':
    main()