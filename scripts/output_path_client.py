#!/usr/bin/env python

import sys
import rospy
from pathplanning.srv import *

def output_path(image):
    rospy.wait_for_service('output_path')
    try:
        output_path = rospy.ServiceProxy('output_path', OutputPath)
        resp1 = output_path(image)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [std_msg height width encoding is_bigendian step data]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        image = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s"%(image)
    rateThrust = output_path(image)
    print ("successfully processed the image into a path")
