#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

def callback(data):
    image = bridge.imgmsg_to_cv2(data, "passthrough")
    image = np.nan_to_num(image)
    cv2.imshow("image", image)
    cv2.waitKey(1)
    print np.max(image), np.min(image)



if __name__ == "__main__":
    rospy.init_node("naive_sound")
    rospy.Subscriber("/depth/image", Image, callback)
    rospy.spin()
