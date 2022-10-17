#!/usr/bin/env python

import rospy
from vector_msgs.msg import Status, ConfigCmd, Battery 

if __name__ == "__main__":
  """
  A playground for testing code to control different aspects of vector
  """
  rospy.init_node('avatar_test')

  stat_pub = rospy.Publisher("/vector/feedback/status", Status, queue_size=1, latch=True)
  cmd_pub = rospy.Publisher("/vector/gp_command",ConfigCmd,queue_size=1, latch=True)
  rospy.Subscriber

  stat = Status()
  cmd = ConfigCmd()

  # Enter Drive mode
  stat.operational_state = 5
  stat_pub.publish(stat)
  rospy.loginfo("Set operational mode to drive mode")
  
  # Stop DTZ mode
  cmd.gp_cmd='GENERAL_PURPOSE_CMD_DISABLE_DTZ_INPUTS'
  cmd.gp_param = 1
  cmd_pub.publish(cmd)
  rospy.loginfo("Disabled DTZ mode")

  rospy.sleep(1.0)
  rospy.loginfo("Configuration completed")