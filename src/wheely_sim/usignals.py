#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

pub = rospy.Publisher('user_commands', Int8, queue_size=10)
rospy.init_node('usignals')
pub.publish(data = 1)
rospy.sleep(0.5)
