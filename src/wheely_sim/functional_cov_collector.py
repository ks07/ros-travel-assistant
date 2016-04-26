#!/usr/bin/env python

import rospy
from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
import std_msgs.msg
import pickle
import sys
import os

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
        if 'FCOV_FILE' in os.environ:
            covname = os.environ['FCOV_FILE']
        else:
            covname = '.fcov'

        # Save the coverage information
        with open(covname, 'w') as covfile:
            pickle.dump(self.coverage, covfile)

        return self.coverage

def combine_coverage(datapaths):
    records = []
    for datapath in datapaths:
        with open(datapath, 'r') as datafile:
            records.append(pickle.load(datafile))

    dest = records[0]
    for covdata in records[1:]:
        for state, labels in covdata.iteritems():
            for label, msgset in labels.iteritems():
                dest[state][label] |= msgset
    return dest

def report(cov):
    # Cross product of signal values and current states
    #self.coverage = {state: {lbl: set() for lbl in SUB_LABELS} for state in struct.children}
    # cov[state][lbl] = set(0,1,...)

    total_points = 0
    covered_points = 0
    
    # Headers
    print '\t'.join(['Active State:','UC0','UC1','UCX','GZ<','GZ>=','CS0','CS1'])

    for state, labels in cov.iteritems():
        # Group begincrossing and crossing
        if state == 'BEGINCROSSING':
           # continue
            pass
        
        # Need to filter/bucket possible values to get an achievable coverage metric
        # User Cmds (buckets for both destinations as well as invalid)
        uc_has_0 = 0 in labels[UC_LABEL]
        uc_has_1 = 1 in labels[UC_LABEL]
        uc_has_inv = bool(labels[UC_LABEL].difference([0,1]))

        total_points += 3
        covered_points += uc_has_0 + uc_has_1 + uc_has_inv

        # Gaze Sensor (buckets for under and inside, ignore invalid)
        # could extend to cover boundary conditions here?
        gz_has_under = any(x < 0.8 for x in labels[GZ_LABEL])
        gz_has_okay  = any(x >= 0.8 for x in labels[GZ_LABEL])

        total_points += 2
        covered_points += gz_has_under + gz_has_okay

        # Crossing Signals (only valid values)
        cs_has_0 = 0 in labels[CS_LABEL]
        cs_has_1 = 1 in labels[CS_LABEL]

        total_points += 2
        covered_points += cs_has_0 + cs_has_1

        print '\t'.join([state] + [str(int(c)) for c in [uc_has_0,uc_has_1,uc_has_inv,gz_has_under,gz_has_okay,cs_has_0,cs_has_1]])

    print 'Covered: ', covered_points
    print 'Total: ', total_points
    print 'Percent Coverage: ', float(covered_points) / float(total_points)

if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == 'report':
        cov = combine_coverage(sys.argv[2:])
        report(cov)
    else:
        rospy.init_node('wheely_fcc')

        fcc = FunctionalCC()
        fcc.run()
