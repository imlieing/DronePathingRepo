#!/usr/bin/env python
import cv2
import numpy as np
# Import required Python code.
import roslib
import rospy
import sys
from rospy import Time

import message_filters
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Image
from autonomous_control.msg import *
from flightgoggles.msg import IRMarkerArray

init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
#gate_list = rospy.get_param("/uav/gate_names")

class image_processing():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.idle_thrust = float(9.81)

        self.input_picture = message_filters.Subscriber("/uav/camera/left/image_rect_color", Image)
        self.ir_marker = message_filters.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray)

        self.pub_vel = rospy.Publisher('processing/rateThrust', RateThrust, queue_size=2)

        ts = message_filters.ApproximateTimeSynchronizer([self.input_picture, self.ir_marker],10, 0.1,allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self,image,ir_marker):

        msg = RateThrust()
        msg.header.frame_id = "uav/imu"
        msg.header.stamp = Time.now()
        msg.thrust.z = 0
        msg.angular_rates.x = 0
        msg.angular_rates.y = 0
        msg.angular_rates.z = 0
        #print("FUCK")
        self.pub_vel.publish(msg)

if __name__ == '__main__':
    rospy.init_node('image_processing')
    try:
        image_processing_node = image_processing()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            #image_processing_node.callback(image_processing_node.input_picture, image_processing_node.ir_marker)

            rate.sleep()
    except rospy.ROSInterruptException: pass
