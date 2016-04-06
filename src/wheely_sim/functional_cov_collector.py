#!/usr/bin/env python

import rospy
from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
import std_msgs.msg
import pickle

UC_LABEL = 'commands'
GZ_LABEL = 'gaze'
CS_LABEL = 'crossing'
SUB_LABELS = [UC_LABEL, GZ_LABEL, CS_LABEL]

SHUTDOWN_CMD = 127

class FunctionalCC(object):
    """ Collects functional coverage for the wheely control logic. The collector
    here checks to see what state the robot was in when certain events (i.e.
    messages) were received. """

    def __init__(self):
        self.ready = False
        self.current_state = None

        # Initial subscribers
        self.smach_sub = rospy.Subscriber('wheely_intro/smach/container_status', SmachContainerStatus, self.smach_status_cb)
        self.smach_info_sub = rospy.Subscriber('wheely_intro/smach/container_structure', SmachContainerStructure, self.smach_info_cb)

        # To be initialised when smach structure received
        self.coverage = None
        self.uc_sub = None
        self.gz_sub = None
        self.cs_sub = None

    def smach_info_cb(self, struct):
        # Only need to read a single message from this topic
        self.smach_info_sub.unregister()

        # Setup the coverage dicts
        self.coverage = {state: {lbl: set() for lbl in SUB_LABELS} for state in struct.children}

        # Start listening for signals
        self.uc_sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, self.uc_callback)
        self.gz_sub = rospy.Subscriber('gaze_sensor', std_msgs.msg.Float32, self.gz_callback)
        self.cs_sub = rospy.Subscriber('crossing_signals', std_msgs.msg.Int8, self.cs_callback)

        print 'Collector initialised.'
        self.ready = True

    def smach_status_cb(self, status):
        print 'status: ',status.active_states
        if len(status.active_states) == 0:
            self.current_state = None
        else:
            self.current_state = status.active_states[0]

    def uc_callback(self, msg):
        if msg.data == SHUTDOWN_CMD:
            rospy.signal_shutdown('Received end of test command.')
        else:
            self.coverage[self.current_state][UC_LABEL].add(msg.data)

    def gz_callback(self, msg):
        self.coverage[self.current_state][GZ_LABEL].add(msg.data)

    def cs_callback(self, msg):
        self.coverage[self.current_state][CS_LABEL].add(msg.data)

    def run(self):
        rospy.spin()

        print 'Collection Finished'
        print self.coverage

        # Save the coverage information
        with open('.fcov', 'w') as covfile:
            pickle.dump(self.coverage, covfile)

if __name__ == '__main__':
    rospy.init_node('wheely_fcc')

    fcc = FunctionalCC()
    fcc.run()
