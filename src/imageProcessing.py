#!/usr/bin/env python
import cv2
import numpy as np
from numpy.linalg import norm
import math
from transforms3d.quaternions import qmult, qinverse, quat2mat
import minieigen
from collections import defaultdict
# Import required Python code.

import roslib
import rospy
import sys
from rospy import Time

import message_filters
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Image
from flightgoggles.msg import IRMarkerArray

def gate_list_centroid_calculator(gate_list):
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
def IR_marker_centroid_calculator(gate_coordinate_list):
    average_x, average_y = 0,0
    for coordinate in gate_coordinate_list:
        average_x += coordinate.x
        average_y += coordinate.y
    average_x = average_x / len(gate_coordinate_list)
    average_y = average_y / len(gate_coordinate_list)
    return average_x, average_y

def IR_marker_cluster_seperator(list_of_IR_markers):
    identified_gates={}
    for i in range(len(list_of_IR_markers.markers)):
        if list_of_IR_markers.markers[i].landmarkID.data in identified_gates:
            identified_gates[list_of_IR_markers.markers[i].landmarkID.data].append(list_of_IR_markers.markers[i])
        else:
            identified_gates[list_of_IR_markers.markers[i].landmarkID.data] = [list_of_IR_markers.markers[i]]
    return identified_gates
##To be used on sub lists IR markers from IR_marker_cluster_seperator.
# def IR_marker_centroid_calculator(sublist_of_IR_markers):
#     total_ir_marker_x = 0
#     total_ir_marker_y = 0
#     for i in range(len(sublist_of_IR_markers)):
#         total_ir_marker_x += sublist_of_IR_markers[i].x
#         total_ir_marker_y += sublist_of_IR_markers[i].y
#     total_ir_marker_x = total_ir_marker_x / len(sublist_of_IR_markers)
#     total_ir_marker_y = total_ir_marker_y / len(sublist_of_IR_markers)
#
#     return total_ir_marker_x, total_ir_marker_y
##I dont think this function will be in the final product
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
    ##This function isnt done, but I dont think its useful either
def q2q_to_rot(q1,q2):
    q1inverted = qinverse(q1)
    ##hopefully this next line is equivalent to q2*q1' = r where r is the rotation quaternion`
    rotation_quaternion = qmult(q2,q1inverted)
    rotation_matrix = quat2mat(rotation_quaternion)
    return rotation_matrix
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
def reorient_to_centroid(init_vector, current_position, current_orientation, target_centroid):
    # find target vector (from current position to gate)
    #current_position = current_pose[:3]
    target_vec3 = [0,0,0]
    for i in range(0,3):
        target_vec3[i] = target_centroid[i] - current_position[i]
    target_vec3 /= np.linalg.norm(target_vec3)
    x,y,z = target_vec3.tolist()
    target_vec3 = minieigen.Vector3(x,y,z)

    #find current orientation Vector
    #w,x,y,z = current_pose[3:].tolist()
    current_quat = minieigen.Quaternion(current_orientation[0],current_orientation[1],current_orientation[2],current_orientation[3])
    init_vec3 = minieigen.Vector3(init_vector)
    current_vec3 = current_quat * init_vec3

    #calculate new quaternion from current vector to target Vector
    new_quat = minieigen.Quaternion()
    new_quat.setFromTwoVectors(current_vec3,target_vec3)

    #compose new quaternions for relative init_orientation
    q = current_quat * new_quat
    q.normalize()
    return [q[3],q[0],q[1],q[2]]

states=['rotating_to_gate','rotating_to_IR_centroid','flying','hover']
image_center_x = 1024/2
image_center_y = 768/2
gates = rospy.get_param("/uav/gate_names")
class image_processing():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        ##To be fixed once we know the structure of the pose message
        init_position = init_pose[:3]
        init_orientation = init_pose[3:]

        self.gate_list = [init_position]
        self.gate_list = self.gate_list + gate_list_centroid_calculator(gates)
        self.idle_thrust = float(9.81)
        #### these parameters all change upon going through a gate
        self.target_gate = 0
        self.current_state = states[0]
        self.current_coords = init_position
        self.current_orientation = init_orientation
        self.target_quaternion = ''
        ####
        self.position1 = [0,0,0]
        self.init_vector = [None]*len(self.current_coords)
        for i in range(len(self.current_coords)):
            self.init_vector[i] = self.current_coords[i] - self.position1[i]
            print(self.init_vector)
        self.init_vector /= np.linalg.norm(self.init_vector)

        self.input_picture = message_filters.Subscriber("/uav/camera/left/image_rect_color", Image)
        self.ir_marker = message_filters.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray)

        self.pub_vel = rospy.Publisher('processing/rateThrust', RateThrust, queue_size=2)

        ts = message_filters.ApproximateTimeSynchronizer([self.input_picture, self.ir_marker],10, 0.1,allow_headerless=True)
        ts.registerCallback(self.callback)


    def successful_gate_callback(self):
        self.target_gate += 1
        self.current_state = states[0]

        self.position1 = self.current_coords
        self.current_coords = self.gate_list[(self.target_gate - 1)]

        for i in range(len(self.current_coords)):
            self.init_vector[i] = self.current_coords[i] - self.position1[i]
            print(self.init_vector)
	    self.init_vector /= linalg.norm(self.init_vector)

	    self.current_orientation = self.target_quaternion
        self.target_quaternion = ''


    def callback(self,image,ir_marker):
        if self.current_state == 'rotating_to_gate':
            self.target_quaternion = reorient_to_centroid(self.init_vector,self.current_coords,self.current_orientation,self.gate_list[self.target_gate+1])
            next_gate_rotmat = q2q_to_rot(self.current_orientation,self.target_quaternion)
            roll_pitch_yaw = rotationMatrixToEulerAngles(next_gate_rotmat)
            print(roll_pitch_yaw)
            #Controls time to rotate
            time_to_rotate = 2
            d = rospy.Duration(time_to_rotate, 0)
            now = rospy.get_rostime()
            end_time = d + now

            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust
            msg.angular_rates.x = roll_pitch_yaw[0]/time_to_rotate
            msg.angular_rates.y = roll_pitch_yaw[1]/time_to_rotate
            msg.angular_rates.z = roll_pitch_yaw[2]/time_to_rotate

            while rospy.get_rostime() < end_time:
                print("ITS AT STATE 1")
                self.pub_vel.publish(msg)
            self.current_state = states[1]

        elif self.current_state == 'rotating_to_IR_centroid':
            # minimum_distance = 99999
            # closest_gate=''
            #
            # for key in range(len(gate_dictionary)):
            #     x,y = IR_marker_centroid_calculator(key)
            #     dist = abs(math.hypot(x - image_center_x, y - image_center_y))
            #     if dist < minimum_distance:
            #         dist = minimum_distance
            #         closest_gate = key
            gate_dictionary = IR_marker_cluster_seperator(ir_marker)
            target_gate_list = gate_dictionary[gates[self.target_gate]]
            #print(gate_dictionary, "REEEEEEEEEEEEEEEEE", target_gate_list)
            x,y = IR_marker_centroid_calculator(target_gate_list)
            delta_x = (-1)*(image_center_x - x)
            delta_y = (-1)*(image_center_y - y)

            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust + 1
            msg.angular_rates.x = 0
            msg.angular_rates.y = delta_y/15
            msg.angular_rates.z = delta_x/15
            print("ITS AT STATE 2")
            self.pub_vel.publish(msg)

            if ((abs(delta_x) < 100) and (abs(delta_y)) < 100):
                self.current_state = states[2]

        elif self.current_state == 'flying':
            gate_dictionary = IR_marker_cluster_seperator(ir_marker)
            target_gate_list = gate_dictionary[gates[self.target_gate]]
            if len(target_gate_list) is 0:
                successful_gate_callback(self)

                msg = RateThrust()
                msg.header.frame_id = "uav/imu"
                msg.header.stamp = Time.now()
                msg.thrust.z = self.idle_thrust
                msg.angular_rates.x = 0
                msg.angular_rates.y = 0
                msg.angular_rates.z = 0
                #print("ITS AT STATE 3")
                self.pub_vel.publish(msg)
            else:
                x,y = IR_marker_centroid_calculator(target_gate_list)
                delta_x = (-1)*(image_center_x - x)
                delta_y = (-1)*(image_center_y - y)

                msg = RateThrust()
                msg.header.frame_id = "uav/imu"
                msg.header.stamp = Time.now()
                msg.thrust.z = self.idle_thrust + 0.1
                msg.angular_rates.x = 100
                msg.angular_rates.y = delta_x / 30
                msg.angular_rates.z = delta_y / 30
                self.pub_vel.publish(msg)
                #print("ITS AT STATE 3")
        elif self.current_state == 'hover':
            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust + 1
            msg.angular_rates.x = 0
            msg.angular_rates.y = 0
            msg.angular_rates.z = 0

if __name__ == '__main__':
    rospy.init_node('image_processing')
    try:
        image_processing_node = image_processing()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: pass
