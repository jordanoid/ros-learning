#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

class image_converter:
    def __init__(self, pub):
        self._pub = pub
        self._bridge = CvBridge()
        
    def publish_image(self, video_dir):
        self._video_dir = video_dir
        while True:
            video_capture = cv2.VideoCapture(self._video_dir)
            try:
                while(video_capture.isOpened()):
                    ret, frame = video_capture.read()
                    self._pub.publish(self._bridge.cv2_to_imgmsg(frame, "bgr8"))
                    time.sleep(0.033)
                video_capture.release()
            except CvBridgeError as e:
                print(e)
            except:
                pass

def main():
    video = "/home/jordano/catkin_ws/src/ros_opencv_integration/video/tennis-ball-video.mp4"
    rospy.init_node("tennis_ball_publisher")
    pub = rospy.Publisher("tennis_ball_image", Image, queue_size=10)
    ic = image_converter(pub)
    ic.publish_image(video)
    rospy.spin()

if __name__ == '__main__':
    main()