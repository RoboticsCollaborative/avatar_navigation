#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8
import serial


# ROS_MASTER_URI [http://10.180.1.111:11311] host is not set to this machine
usb_string = '/dev/ttyACM0'
threshold = 300 #no pressure is nominally 0, max reading is 1024

WAITING = 0
INITIALIZING = 1
DRIVING = 2

def status_pub():
    pub = rospy.Publisher('pressure_status', Bool, queue_size=10)
    stat_pub = rospy.Publisher('driving_status', Int8, queue_size=10)
    rospy.init_node('pressure_sensor', anonymous=True)
    ser = serial.Serial(usb_string, 9600, timeout = 1)
    string = ser.readline()
    driving = False
    counter = 0
    counter_thresh = 30

    while not rospy.is_shutdown():
        string = ser.readline()
        stat_msg = Int8(WAITING)

        try:
            value = int(string[:-2])
        except:
            rospy.logwarn('footpad_status_publisher failed to convert data: {}'.format(string))
            value = -1
        
        if value > threshold:
            if not driving:
                counter += 1
                stat_msg = Int8(INITIALIZING)
            if counter > counter_thresh:
                driving = True
                stat_msg = Int8(DRIVING)
        else:
            driving = False
            counter = 0
        #publish
        msg = Bool(driving)
        pub.publish(msg)
        stat_pub.publish(stat_msg)

        

if __name__ == '__main__':
    try:
        status_pub()
    except rospy.ROSInterruptException:
        pass
