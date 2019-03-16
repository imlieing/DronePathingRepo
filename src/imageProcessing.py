#!/usr/bin/env python
import cv2
import numpy as np
import math
from transforms3d.quaternions import axangle2quat, qmult
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

gates = rospy.get_param("/uav/gate_names")
###This for loop creates a list of all the gates corresponding centroid coordinates
def gate_centroid_calculator(gate_list):
    gate_centroid_coordinates = [None]*len(gate_list)
    for i in range(len(gate_list)):
        #getting current gate name and rosparam
        gate_string = gate_list[i].strip()
        gate_coordinates = rospy.get_param("/uav/" + gate_string + "/nominal_location")

        ##calculating centroid here
        average_x, average_y, average_z = 0,0,0
        for coordinate in gate_coordinates:
            average_x += coordinate[0]
            average_y += coordinate[1]
            average_z += coordinate[2]
        average_x = average_x / 4
        average_y = average_y / 4
        average_z = average_z / 4

        gate_centroid_coordinates[i] = [average_x, average_y, average_z]
    return gate_centroid_coordinates

def coordinate_angle_and_distance_calculator(point1_list, point2_list):
    x1 = point1_list[0]
    y1 = point1_list[1]
    z1 = point1_list[2]
    x2 = point2_list[0]
    y2 = point2_list[1]
    z3 = point2_list[2]

    magnitude = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**(1/2)

    numerator_for_arccos = x2*x1 + y2*y1 + z2*z1
    denominator_for_arccos = ((x1**2 + y1**2 + z1**2)*(x2**2 + y2**2 + z2**2))**(1/2)

    direction = math.acos(numerator_for_arccos/denominator_for_arccos)


class image_processing():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        self.current_gate = 0

        #To be commented out, doesnt belong here?
        self.idle_thrust = float(9.81)
        ##

        self.input_picture = message_filters.Subscriber("/uav/camera/left/image_rect_color", Image)
        self.ir_marker = message_filters.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray)

        self.pub_vel = rospy.Publisher('processing/rateThrust', RateThrust, queue_size=2)

        ts = message_filters.ApproximateTimeSynchronizer([self.input_picture, self.ir_marker],10, 0.1,allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self,image,ir_marker):
        print("durr")
        print(ir_marker)
        print("FUCK")
        msg = RateThrust()
        msg.header.frame_id = "uav/imu"
        msg.header.stamp = Time.now()
        msg.thrust.z = 20
        msg.angular_rates.x = 0
        msg.angular_rates.y = 0
        msg.angular_rates.z = 0
        #print("FUCK")
        self.pub_vel.publish(msg)

    def successful_gate_callback(self):
        self.gate_iterator += 1
        self.init_pose = rospy.get_param("")

if __name__ == '__main__':
    rospy.init_node('image_processing')
    gate_list = gate_centroid_calculator(gates)

    try:
        image_processing_node = image_processing()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            #image_processing_node.callback(image_processing_node.input_picture, image_processing_node.ir_marker)

            rate.sleep()
    except rospy.ROSInterruptException: pass
