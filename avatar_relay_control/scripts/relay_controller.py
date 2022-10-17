#!/usr/bin/env python
import rospy
import time
import sys
from std_msgs.msg import Bool, Header
from avatar_relay_control.msg import RelayStatus
from avatar_relay_control.srv import SetRelay, SetRelayResponse
from relay import RelayBoard, get_port_from_ser

#TODO - Startup Behavior - Turn on automatically?
# Shutdown Behavior - Keep current configutation?

class RelayController():
    def __init__(self):

        self.namespace = rospy.get_namespace()
        self.serial_number = sys.argv[1]
        self.num_relays = int(sys.argv[2])
        self.serial_port = get_port_from_ser(self.serial_number)

        # if self.serial_port == "":
        #     rospy.logerror("NO Serial Device found for ID number: {}. Try running $ python3 -m serial.tools.list_ports -v to check for the device".format(self.serial_number))

        self.controller = RelayBoard(self.serial_port, self.num_relays)

        rospy.loginfo("starting relay controller")
        rospy.init_node('relay_controller')
        self.pub = rospy.Publisher('relay_status', RelayStatus, queue_size=10)
        self.service = rospy.Service('set_relay_status', SetRelay, self.service_callback)
        rate = rospy.Rate(1)

        #turn off all relays to start
        for i in range(self.num_relays):
            self.controller.set_relay(i,0)

        while not rospy.is_shutdown():
            # status = self.controller.get_status()

            #publish status
            # h = Header()
            # h.stamp = rospy.Time.now()
            # status_msg = RelayStatus()
            # status_msg.header = h
            # status_msg.Relay_1 = Bool(status[0])
            # status_msg.Relay_2 = Bool(status[1])
            # status_msg.Relay_3 = Bool(status[2])
            # status_msg.Relay_4 = Bool(status[3])
            # self.pub.publish(status_msg)
            rate.sleep()

    def service_callback(self, request):
        #check for valid index
        if request.relay_number >= self.num_relays:
            rospy.logwarn("Bad Relay Index: {}. Relay Not Set.".format(request.relay_number))
            return SetRelayResponse(False)

        #attempt to set relays
        return SetRelayResponse(self.controller.set_relay(request.relay_number, request.relay_status.data))

    def get_status(self):
        return [False, False, False, False]






if __name__ == "__main__":
    controler = RelayController()
