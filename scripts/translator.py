#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

depthpub = rospy.Publisher("/camera/depth_registered/image_raw", Image, queue_size=1)
rgbpub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=1)

def depth_callback(data):
    depthpub.publish(data)

def rgb_callback(data):
    rgbpub.publish(data)

if __name__ == "__main__":
    rospy.init_node("naive_sound")
    rospy.Subscriber("/depth/image", Image, depth_callback)
    rospy.Subscriber("/rgb/image", Image, rgb_callback)
    rospy.spin()
