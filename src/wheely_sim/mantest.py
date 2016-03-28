#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int8, Float32

utype = sys.argv[1]
udata = sys.argv[2]

rospy.init_node('usignals')

if utype == 'u':
	pub = rospy.Publisher('user_commands', Int8, queue_size=1)
        udata = int(udata)
elif utype == 'c':
	pub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
        udata = int(udata)
elif utype == 'g':
        pub = rospy.Publisher('gaze_sensor', Float32, queue_size=1)
        udata = float(udata)
rospy.sleep(0.2)
pub.publish(data = udata)
rospy.sleep(0.1)
