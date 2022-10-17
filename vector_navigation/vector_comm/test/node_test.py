#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def lambda_cb(topic):
  return lambda msg: print(msg.data)


rospy.init_node("test")
rospy.Subscriber("/test", String, lambda_cb("/test"))

rospy.spin()