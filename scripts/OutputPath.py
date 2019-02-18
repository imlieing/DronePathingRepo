#!/usr/bin/env python

import rospy
from pathplanning.srv import *
def handle_output_path(req):
	print ("returning path instructions")
	return OutputPathsResponse(header_output, angular_rates_output, thrust_output)
def output_path_server():
        rospy.init_node('output_path_server')
        s = rospy.Service('output_path', OutputPath, handle_output_path)
        print ("Ready to output path.")
        rospy.spin()

if __name__ == "__main__":
        output_path_server()
