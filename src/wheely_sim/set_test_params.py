#!/usr/bin/env python

import rospy
rospy.init_node('set_test_params')

# Configuration
enable_island = True
light_response_time = [0,10000]
light_crossing_time = [5000,15000]
gaze_wait_time      = [0,500]
gaze_wait_time_2    = [0,500]
test_end_time       = [10000,10000]
new_cmd_wait_time   = [0,10000]
user_commands       = [0,2.5]
crossing_signals    = [0,2]
gaze_sensor         = [0.6,1.0]


# Set the parameters
if enable_island:
    rospy.set_param('/wheely_sim/has_island', 1)
    rospy.set_param('/wheely_sim/road_areas',[2.75,0.3,-0.3,-2.85])
else:
    rospy.set_param('/wheely_sim/has_island',0)
    rospy.set_param('/wheely_sim/road_areas',[2.75,-2.85]) # high to low
rospy.set_param('/wheely_sim/map_bounds',[-3.2,3.2]) # low then high

rospy.set_param('/bdiparam/light_response_time',light_response_time)
rospy.set_param('/bdiparam/light_crossing_time',light_crossing_time)
rospy.set_param('/bdiparam/gazewait',gaze_wait_time)
rospy.set_param('/bdiparam/gazewait2',gaze_wait_time_2)
rospy.set_param('/bdiparam/testend',test_end_time)
rospy.set_param('/bdiparam/newcmdwait',new_cmd_wait_time)

rospy.set_param('/bdiparam/user_commands',user_commands)
rospy.set_param('/bdiparam/crossing_signals',crossing_signals)
rospy.set_param('/bdiparam/gaze_sensor',gaze_sensor)
