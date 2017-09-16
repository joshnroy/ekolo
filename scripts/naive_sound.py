#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

def callback(data):
    image = bridge.imgmsg_to_cv2(data, "passthrough")
    cv2.imshow('image', image)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("naive_sound")
    rospy.Subscriber("/depth/image", Image, callback)
    rospy.spin()
