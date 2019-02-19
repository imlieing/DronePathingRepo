#!/usr/bin/env python

import rospy
from mav_msgs.msg import RateThrust

from pathplanning.srv import *
def handle_output_path(req):
	print ("returning path instructions")
	msg = RateThrust()
	msg.header.frame_id = "uav/imu"
	msg.header.stamp = Time.now()
	msg.thrust.z = self.idleThrust + 1;
	msg.angular_rates.x = 0.05
	msg.angular_rates.y = 0.05
	msg.angular_rates.z = 0.05
	return OutputPathsResponse(msg)
def output_path_server():
        rospy.init_node('output_path_server')
        s = rospy.Service('output_path', OutputPath, handle_output_path)
        print ("Ready to output path.")
        rospy.spin()

if __name__ == "__main__":
	output_path_server()
