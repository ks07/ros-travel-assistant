#!/usr/bin/env python

# This script changes road config, at given prob. Needs to run before launching simulation modules

import rospy
import random

def set_params():
    # Set the parameters
    if random.random() < island_prob:
        print 'Island Enabled'
        rospy.set_param('/wheely_sim/has_island', 1)
        rospy.set_param('/wheely_sim/road_areas',[2.75,0.3,-0.3,-2.85])
    else:
        print 'Island Disabled'
        rospy.set_param('/wheely_sim/has_island',0)
        rospy.set_param('/wheely_sim/road_areas',[2.75,-2.85]) # high to low
    rospy.set_param('/wheely_sim/map_bounds',[-3.2,3.2]) # low then high

if __name__ == '__main__':
    island_prob = 0.5
    rospy.init_node('cprg_test_setup')
    set_params()
