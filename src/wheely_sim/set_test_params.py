#!/usr/bin/env python

import rospy
rospy.init_node('set_test_params')

# Configuration
enable_island = True
light_response_time = [0,10000]
light_crossing_time = [0,10000]


# Set the parameters
if enable_island:
    rospy.set_param('/wheely_sim/has_island', 1)
    rospy.set_param('/wheely_sim/road_areas',[2.75,0.3,-0.3,-2.85])
else:
    rospy.set_param('/wheely_sim/has_island',0)
    rospy.set_param('/wheely_sim/road_areas',[2.75,-2.85])

rospy.set_param('/bdiparam/light_response_time',light_response_time)
rospy.set_param('/bdiparam/light_crossing_time',light_crossing_time)
