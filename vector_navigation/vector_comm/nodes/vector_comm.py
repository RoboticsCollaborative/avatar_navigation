#! /usr/bin/env python3

from json import dumps, loads
import queue
from rospy_websocker_client import ws_client as ros_ws
from math import sin, cos
from time import sleep
import rospy
from vector_msgs.msg import Status, ConfigCmd, Battery 
from geometry_msgs.msg import Twist


class VectorRelay(ros_ws):

    def __init__(self, address, port):
        super().__init__(address, port)

        # Relay all operator subscribers here
        self.subscribe("/vector/feedback/status", Status(), queue_length=1)
        # subscribe to laserscan here
        rospy.loginfo("Receiving from status topic")

        # Relay all operator publishers here
        self.gp_command_sub = rospy.Subscriber("/vector/gp_command", ConfigCmd, self.gp_command_cb, queue_size=1)
        self.cmd_vel_sub    = rospy.Subscriber("/vector/cmd_vel", Twist, \
                                                    self.relay_cb("/vector/cmd_vel"), queue_size=10)
        self.status_sub     = rospy.Subscriber("/vector/feedback/status", Status, \
                                                    self.relay_cb("/vector/feedback/status"), queue_size=1)

        self.connect()

    def relay_cb(self, topic):
        return lambda msg: self.publish(topic, msg)

    def gp_command_cb(self, msg):
        self.publish('/vector/gp_command', msg)
        rospy.loginfo("Published gp commands from operator")

if __name__ == "__main__":
  rospy.init_node("vector_comm")

  # Vector's default rosbridge server ip and port
  vc = VectorRelay('10.66.171.2', 9090)

  rospy.spin()
  
