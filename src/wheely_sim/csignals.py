#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

pub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
rospy.init_node('csignals')
pub.publish(data = 99)
rospy.sleep(0.5)
