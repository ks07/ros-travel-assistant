#!/usr/bin/env python

import rospy
rospy.init_node('set_test_params')

rospy.set_param('/bdiparam/light_response_time',5000)
