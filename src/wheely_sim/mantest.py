#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int8

utype = sys.argv[1]
udata = sys.argv[2]

rospy.init_node('usignals')

if utype == 'u':
	pub = rospy.Publisher('user_commands', Int8, queue_size=1)
elif utype == 'c':
	pub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
rospy.sleep(0.2)
pub.publish(data = int(udata))
rospy.sleep(0.1)
