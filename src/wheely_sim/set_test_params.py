#!/usr/bin/env python

import rospy
rospy.init_node('set_test_params')

rospy.set_param('/wheely_sim/has_island',0)

rospy.set_param('/bdiparam/light_response_time',[0,10000])
rospy.set_param('/bdiparam/light_crossing_time',[0,10000])
