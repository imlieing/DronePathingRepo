#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from rospy import Time

from mav_msgs.msg import RateThrust
from autonomous_control.msg import *

class image_processing():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.init_node('image_listener', anonymous=True)

        rospy.Subscriber("/bounding_box_camera/RGB", String, callback)

    def control(self):


if __name__ == '__main__':
    rospy.init_node('image_processing)
    try:
        image_processing_node = image_processing()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            image_processing_node.control()
            rate.sleep()
    except rospy.ROSInterruptException: pass
