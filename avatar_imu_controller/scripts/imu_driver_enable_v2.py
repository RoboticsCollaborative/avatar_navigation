#!/usr/bin/env python3
'''script to turn a vector nav IMU orientation into a twist velocty command'''

import rospy
from numpy import cos, sin, deg2rad, unwrap
import numpy as np
import serial
import time
from copy import deepcopy
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import tf
import csv
from tf.transformations import *

# sample_string = '$VNYMR,-004.328,+001.357,+178.257,+00.2219,-00.0720,+00.4652,+00.226,-00.321,+10.043,-00.001188,+00.001866,-00.007661*6C   '

#settings
deadband = deg2rad(5)
roll_deadband = deg2rad(4)
pitch_deadband = deg2rad(4)
yaw_deadband = deg2rad(2)

class TiltController():
    def __init__(self, usb_string, max_x_vel, max_y_vel, max_yaw_vel):
        #initilaize serial object
        self.ser = serial.Serial(usb_string, 115200, timeout = 1)

        self.max_x_vel = max_x_vel
        self.max_rev_vel = max_x_vel/2
        self.max_y_vel = max_y_vel
        self.max_yaw_vel = max_yaw_vel

        self.max_roll_angle = deg2rad(15)
        self.max_pitch_angle = deg2rad(15)
        self.max_yaw_angle = deg2rad(10)

        self.ref_roll = 0
        self.ref_pitch = 0
        self.ref_yaw = 0

        self.quat = np.array([0, 0, 0, 1.0])
        #initilaize publisher object
        # pub_imu = rospy.Publisher('imu_pub', Imu, queue_size = 10)
        # pub_mag = rospy.Publisher('mag_pub', MagneticField, queue_size = 10)
        self.pub_twist = rospy.Publisher('/vector/cmd_vel', Twist, queue_size = 1)
        self.pub_quat = rospy.Publisher('imu_orientation', Quaternion, queue_size=1)
        self.pub_mag = rospy.Publisher('magnetic_orientation', Vector3, queue_size=1)
        self.sub = rospy.Subscriber('/pressure_status', Bool, self.enable_callback)
        self.enable = False
        # self.br = tf.TransformBroadcaster()

        self.ref_orientation = np.array([0, 0, 0, 1.])
        self.ref_mag_vector = np.array([0, 0, 1.])
        self.ref_orientation_inv = np.array([0., 0., 0., -1.0])

        #initialize node
        rospy.init_node('tilt_controller', anonymous = False)
        time.sleep(0.1)

    def enable_callback(self, data):
        if not self.enable and data.data:
            self.update_ref_to_current()
            self.enable = True
        elif self.enable and not data.data:
            self.enable = False

    def update_ref_to_current(self):
        self.ref_roll, self.ref_pitch, self.ref_yaw = euler_from_quaternion(self.quat)

    def run(self):
        first_time = True
        self.ser.reset_input_buffer() #clear buffer

        while not rospy.is_shutdown() :
            #attempts to read in imu messge if none found, skips
            # ser_string = sample_string #no buffer checking, could be an issue
            # print('Buffer Size ={}'.format(ser.inWaiting())) #check for buffer overrun
            ser_string = self.ser.readline()
            if (float(serial.__version__) > 3):
                ser_string = ser_string.decode("utf-8")
            time_stamp = rospy.Time.now()

            if ser_string == '': #if nothing is recieved, continue the loop
                rospy.loginfo('No Serial connection to IMU...')
                # time.sleep(0.5)
                continue

            #check for the correct string identifier
            if ser_string[:6] == '$VNYMR':
                #checksum verification
                if verify_checksum(ser_string):
                    #attmpe to parse line
                    format_good, data_dict = parse_imu(ser_string)
                    if format_good:
                        #generate messages
                        # imu_msg, mag_msg = build_messages(data_dict, time_stamp)
                        self.quat = parse_orientation(data_dict)
                        mag_vec = parse_mag(data_dict)
                        mag_vec /= np.linalg.norm(mag_vec)

                        #calculate quaternion difference
                        # quaternion_difference = quaternion_multiply(quat, self.ref_orientation_inv)
                        # roll, pitch, yaw = euler_from_quaternion(quaternion_difference)
                        roll, pitch, yaw = euler_from_quaternion(self.quat)
                        if first_time == True:
                            first_time = False
                            self.ref_orientation = deepcopy(self.quat)
                            self.ref_orientation_inv = deepcopy(self.quat)
                            self.ref_orientation_inv[3] *= -1
                            self.ref_roll, self.ref_pitch, self.ref_yaw = roll, pitch, yaw
                            ref_mag_vector = deepcopy(mag_vec)
                            ref_mag_vector /= np.linalg.norm(ref_mag_vector)
                            prev_roll = roll
                            prev_pitch = pitch
                            prev_yaw = yaw

                        roll = unwrap([prev_roll,roll])[1]
                        pitch = unwrap([prev_pitch,pitch])[1]
                        yaw = unwrap([prev_yaw,yaw])[1]
                        prev_roll = roll
                        prev_pitch = pitch
                        prev_yaw = yaw
                        
                        #apply zero
                        roll, pitch, yaw = roll - self.ref_roll, pitch - self.ref_pitch, yaw - self.ref_yaw

                        #remap axes
                        roll = roll
                        pitch = -pitch
                        yaw = -yaw
                        # print(roll)
                        #apply deadband and max angle clip
                        roll = np.clip(apply_deadband(roll, -roll_deadband, roll_deadband), -self.max_roll_angle, self.max_roll_angle)
                        pitch = np.clip(apply_deadband(pitch, -pitch_deadband, pitch_deadband), -self.max_pitch_angle, self.max_pitch_angle)
                        yaw = np.clip(apply_deadband(yaw, -yaw_deadband, yaw_deadband), -self.max_yaw_angle, self.max_yaw_angle)
                        # print(roll)
                        #normalize angle to range
                        # norm_roll = roll/(self.max_roll_angle-roll_deadband)
                        norm_roll = roll/(self.max_roll_angle)
                        norm_pitch = pitch/(self.max_pitch_angle)
                        norm_yaw = yaw/(self.max_yaw_angle)
                        # print(norm_roll)
                        #convert to linear velocity
                        if norm_roll > 0:
                            x_vel = norm_roll*self.max_x_vel
                        else:
                            x_vel = norm_roll*self.max_rev_vel
                        y_vel = norm_pitch*self.max_y_vel
                        yaw_vel = norm_yaw*self.max_yaw_vel

                        # print(x_vel)
                        # print("******")
                        #calculate twist message
                        twist_message = Twist()
                        if self.enable:
                            # twist_message.linear.x = roll
                            # twist_message.linear.y = pitch
                            # twist_message.linear.z = 0
                            # twist_message.angular.x = 0
                            # twist_message.angular.y = 0
                            # twist_message.angular.z = yaw

                            twist_message.linear.x = x_vel
                            twist_message.linear.y = y_vel
                            # twist_message.linear.x = -y_vel
                            # twist_message.linear.y = -x_vel
                            twist_message.linear.z = 0
                            twist_message.angular.x = 0
                            twist_message.angular.y = 0
                            twist_message.angular.z = yaw_vel

                        #calculate magnetic difference
                        # cross_product = np.cross(mag_vec, ref_mag_vector)
                        # angle = np.linalg.norm(cross_product)
                        # angle = np.arcsin(np.linalg.norm(cross_product))


                        #project ref mag vector into current imu orientation frame
                        #remove z-component
                        #calculate angular difference

                        #publish messages
                        # pub_imu.publish(imu_msg)
                        # pub_mag.publish(mag_msg)
                        # quat_msg = Quaternion(x = self.quat[0], y = self.quat[1], z = self.quat[2], w = self.quat[3])
                        # mag_msg = Vector3(x = mag_vec[0], y = mag_vec[1], z = mag_vec[2])
                        # self.pub_quat.publish(quat_msg)
                        # self.pub_mag.publish(mag_msg)
                        self.pub_twist.publish(twist_message)



                    else: #string does not match expected format
                        rospy.loginfo('Bad Checksum')
                        continue
            else: #not the right sentence
                rospy.loginfo('Incorrect IMU Header')
                continue

def apply_deadband(x, lower, upper):
    assert(upper > lower)
    if lower < x < upper:
        x = 0
    elif x <= lower:
        x = x - lower
    else:
        x = x - upper
    return x

def verify_checksum(string):
    '''Checks the string against the verify_checksum
    Input: string "Full gps string"
    Output: T/F '''

    #pull out the relevant part of the string
    #split the string on *
    strings = string.split('*')
    if len(strings) == 2:
        string_sum = strings[0][1:] #the message minus leading $
        string_check = strings[1] #the checksum without *, may have traing spaces or linefeed

        #calculate checksum
        str_xor =  0
        for character in string_sum:
            str_xor = str_xor ^ ord(character)

        #verify
        if int(string_check,16) == str_xor:
            return True
        else:
            return False
    else: #too many *, or no *
        return False

def parse_imu(string):
    '''Reads imu string and outputs info in a dict
    Input: full imu string
    Output: data dict'''
    str_list = string.replace('*',',').split(',')
    imu_dict = {}
    format_good = True
    #parse * try block protects against weird strings
    try:
        imu_dict['yaw'] = float(str_list[1])    #degrees
        imu_dict['pitch'] = float(str_list[2])  #degrees
        imu_dict['roll'] = float(str_list[3])   #degrees
        imu_dict['mag_x'] = float(str_list[4])  #gaus
        imu_dict['mag_y'] = float(str_list[5])  #gaus
        imu_dict['mag_z'] = float(str_list[6])  #gaus
        imu_dict['accel_x'] = float(str_list[7]) #m/s^2
        imu_dict['accel_y'] = float(str_list[8]) #m/s^2
        imu_dict['accel_z'] = float(str_list[9]) #m/s^2
        imu_dict['gyro_x'] = float(str_list[10]) #rad/sec
        imu_dict['gyro_y'] = float(str_list[11]) #rad/sec
        imu_dict['gyro_z'] = float(str_list[12]) #rad/sec
    except ValueError:
        format_good = False
        #prints if there is an error
        print('Checksum OK, but failed to convert to parse string:')
        print(string)

    return format_good, imu_dict

def parse_orientation(imu_dict):
    #quaternion conversion
    cy = cos(deg2rad(imu_dict['yaw'] * 0.5));
    sy = sin(deg2rad(imu_dict['yaw'] * 0.5));
    cp = cos(deg2rad(imu_dict['pitch'] * 0.5));
    sp = sin(deg2rad(imu_dict['pitch'] * 0.5));
    cr = cos(deg2rad(imu_dict['roll'] * 0.5));
    sr = sin(deg2rad(imu_dict['roll'] * 0.5));
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    return np.array([qx, qy, qz, qw])

def parse_mag(imu_dict):
    return np.array([imu_dict['mag_x'], imu_dict['mag_y'], imu_dict['mag_z']])
def wrap_2pi(num):
    if num<0:
        num = 2*np.pi + num
    return num
def build_messages(imu_dict, time_stamp):
    '''Takes a dict generated by parse_imu() and retuns
    a complete IMU.msg message object'''

    #quaternion conversion
    cy = cos(deg2rad(imu_dict['yaw'] * 0.5));
    sy = sin(deg2rad(imu_dict['yaw'] * 0.5));
    cp = cos(deg2rad(imu_dict['pitch'] * 0.5));
    sp = sin(deg2rad(imu_dict['pitch'] * 0.5));
    cr = cos(deg2rad(imu_dict['roll'] * 0.5));
    sr = sin(deg2rad(imu_dict['roll'] * 0.5));
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    # print([imu_dict['yaw'],imu_dict['pitch'],imu_dict['roll']])
    # print([qx,qy,qz,qw])
    #build imu message
    orient =  Quaternion(x=qx,
                         y=qy,
                         z=qz,
                         w=qw)
    accel =   Vector3(x=imu_dict['accel_x'],
                      y =imu_dict['accel_y'],
                      z =imu_dict['accel_z'])
    ang_vel = Vector3(x=imu_dict['gyro_x'],
                      y=imu_dict['gyro_y'],
                      z=imu_dict['gyro_z'])
    imu_msg = Imu(header = Header(stamp = time_stamp),
                  orientation=orient,
                  linear_acceleration = accel,
                  angular_velocity=ang_vel)

    #build magnetic message
    #reuses the header to have the same time
    mag_fld = Vector3(x = imu_dict['mag_x'],
                      y = imu_dict['mag_y'],
                      z = imu_dict['mag_z'])
    mag_msg = MagneticField(header = Header(stamp = time_stamp),
                            magnetic_field = mag_fld)
    return imu_msg, mag_msg

# def main():
#     '''Reads in serial messages and publishes them'''

#     #initilaize serial object
#     ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)

#     #initilaize publisher object
#     pub_imu = rospy.Publisher('imu_pub', Imu, queue_size = 10)
#     pub_mag = rospy.Publisher('mag_pub', MagneticField, queue_size = 10)
#     pub_twist = rospy.Publisher('imu_twist', Twist, queue_size = 1)

#     #initialize node
#     rospy.init_node('tilt_controller', anonymous = True)
#     # rate = rospy.Rate(100)
#     #no rate specified, will publish at the imu rate

#     # with open('raw_data.csv', 'w') as csvfile:
#     #     wrtr = csv.writer(csvfile)
#     #     wrtr.writerow(['Time Stamp', 'String'])

#     #loop to continue reading in GPS
#     count = 0
#     while not rospy.is_shutdown() :
#         #attempts to read in GPS messge if none found, skips
#         # ser_string = sample_string #no buffer checking, could be an issue
#         # print('Buffer Size ={}'.format(ser.inWaiting())) #check for buffer overrun
#         ser_string = ser.readline()
#         time_stamp = rospy.Time.now()
#         # #save data to log textfile just in case
#         # with open('raw_data.csv', 'a') as csvfile:
#         #     wrtr = csv.writer(csvfile)
#         #     wrtr.writerow([time_stamp, ser_string])

#         if ser_string == '': #if nothing is recieved, continue the loop
#             rospy.loginfo('No Serial connection to IMU...')
#             # time.sleep(0.5)
#             continue

#         #check for the correct string identifier
#         if ser_string[:6] == '$VNYMR':
#             #checksum verification
#             if verify_checksum(ser_string):
#                 #attmpe to parse line
#                 format_good, data_dict = parse_imu(ser_string)
#                 if format_good:
#                     #generate messages
#                     imu_msg, mag_msg = build_messages(data_dict, time_stamp)
#                     #publish messages
#                     count+=1
#                     rospy.loginfo('Publishing {}'.format(count))
#                     pub_imu.publish(imu_msg)
#                     pub_mag.publish(mag_msg)
#                 else: #string does not match expected format
#                     rospy.loginfo('Bad Checksum')
#                     continue
#         else: #not the right sentence
#             rospy.loginfo('Incorrect IMU Header')
#             continue

#         # rate.sleep() #for testing only

if __name__ == '__main__':

    serial_string = '/dev/ttyUSB0'
    deadband = deg2rad(5)
    yaw_deadband = deg2rad(5)
    tilt_joystick = TiltController(serial_string, 0.4, 0.15, 0.2)
    tilt_joystick.run()

    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass
