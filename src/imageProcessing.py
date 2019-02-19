#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from rospy import Time

from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Image
from autonomous_control.msg import *
from pathplanning.srv import *

class image_processing():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.idleThrust = float(9.81)
        self.pub_vel = rospy.Publisher('processing/rateThrust', RateThrust, queue_size=2)
        self.input_picture = rospy.Subscriber("/bounding_box_camera/RGB", Image, OutputPath)

    def control(self):
        msg = RateThrust()
        msg.header.frame_id = "uav/imu"

        #output_path = rospy.ServiceProxy('output_path', OutputPath)
        #output_path(self.input_picture)

        msg.header.stamp = Time.now()

        msg.thrust.z = self.idleThrust + 1;
        msg.angular_rates.x = 0.05
        msg.angular_rates.y = 0.05
        msg.angular_rates.z = 0.05

        self.pub_vel.publish(msg)


if __name__ == '__main__':
    rospy.init_node('image_processing')
    try:
        image_processing_node = image_processing()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            image_processing_node.control()
            rate.sleep()
    except rospy.ROSInterruptException: pass
