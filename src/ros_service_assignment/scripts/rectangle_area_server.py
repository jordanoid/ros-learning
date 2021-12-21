#!/usr/bin/env python

from ros_service_assignment.srv import *
import rospy

def handle_area(req):
    resp = RectangleAreaServiceResponse()
    resp.area = req.width * req.height
    return resp

def main():
    rospy.init_node("rectangle_area_server", anonymous=True)
    s = rospy.Service("get_rectangle_area", RectangleAreaService, handle_area)
    rospy.loginfo("Rectangle Area Service Server On")
    rospy.spin()

if __name__ == "__main__":
    main()