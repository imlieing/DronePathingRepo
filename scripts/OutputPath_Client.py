import sys
import rospy
from pathplanning.srv import *

def output_path(std_msg,height,width,encoding,is_bigendian,step,data):
    rospy.wait_for_service('output_path')
    try:
        output_path = rospy.ServiceProxy('output_path', OutputPath)
        resp1 = output_path(std_msg,height,width,encoding,is_bigendian,step,data)
        return resp1.header, resp1.angular_rates, resp1.thrust
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [std_msg height width encoding is_bigendian step data]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 8:
        std_msg = int(sys.argv[1])
        height = int(sys.argv[2])
        width = int(sys.argv[3])
        encoding = int(sys.argv[4])
        is_bigendian = int(sys.argv[5])
        step = int(sys.argv[6])
        data = int(sys.argv[7])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s+%s+%s+%s+%s+%s"%(std_msg, height,width,encoding,is_bigendian,step,data)
    header_output, angular_rates_output, thrust_output = output_path(std_msg, height,width,encoding,is_bigendian,step,data)
    print ("successfully processed the image into a path")
