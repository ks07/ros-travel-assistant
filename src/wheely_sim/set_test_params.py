#!/usr/bin/env python

import rospy
import random
import sys
rospy.init_node('set_test_params')

# Configuration
if len(sys.argv) > 1 and sys.argv[1] == 'y':
    enable_island = True
elif len(sys.argv) > 1 and sys.argv[1] == 'n':
    enable_island = False
elif random.getrandbits(1):
    enable_island = True
else:
    enable_island = False
light_response_time = [0,100]
light_crossing_time = [100,5500]
gaze_wait_time      = [0,50]
gaze_wait_time_2    = [0,50]
test_end_time       = [10000,10000]
new_cmd_wait_time   = [0,4000]
user_commands       = [0,2.5]
crossing_signals    = [0,2]
gaze_sensor         = [0.7,1.0]


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
