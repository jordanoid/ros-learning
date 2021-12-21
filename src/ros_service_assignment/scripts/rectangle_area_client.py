#!/usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import *

def rectangle_area_client(height, width):
    rospy.wait_for_service("get_rectangle_area")
    try:
        get_area = rospy.ServiceProxy("get_rectangle_area", RectangleAreaService)
        print("Service called")
        resp = get_area(height, width)
        return resp.area
    except rospy.ROSInterruptException:
        print("Service call failed")
    
if __name__ == '__main__':
    if len(sys.argv) == 3:
        height = int(sys.argv[1])
        width = int(sys.argv[2])
    else:
        print("%s [height width]"%sys.argv[0])
        sys.exit(1)
    print("Requesting area")
    area = rectangle_area_client(height, width)
    print("Area = %s"%area)