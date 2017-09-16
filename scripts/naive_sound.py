#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def callback(data):
    print "got image"

if __name__ == "__main__":
    rospy.init_node("naive_sound")
    rospy.Subscriber("/depth/image", Image, callback)
    rospy.spin()
