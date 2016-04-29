#!/usr/bin/env python

# This script runs constrained pseudorandomly generated tests, to compare and contrast with BDI

import sys
import rospy
import threading
import random
from std_msgs.msg import Int8, Float32

upub = rospy.Publisher('user_commands', Int8, queue_size=1)
cpub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
gpub = rospy.Publisher('gaze_sensor', Float32, queue_size=1)

def set_constraints():
    # Sets the constraints to use for generation
    global stim_count
    stim_count = (3,30)
    global delay_constraint
    delay_constraint = (0.0,7.0)
    global user_invalid_prob
    user_invalid_prob = 0.1
    global user_inv_command
    user_inv_command = 2
    global user_good_command
    user_good_command = (0,1)
    global crossing_signal
    crossing_signal = (0,1)
    global gaze_bad_prob
    gaze_bad_prob = 0.2
    global gaze_bad
    gaze_bad = (0.0,0.79)
    global gaze_good
    gaze_good = (0.8,1.0)
    # Sequence bias
    global std_seq_bias
    std_seq_bias = 0.4

def delay():
    # Adds a random delay
    t = random.uniform(*delay_constraint)
    print 'Sleep: ', t
    rospy.sleep(t)

def make_stimuli(type):
    # Uses constraints to pick params and send desired type of stimuli
    if type == 'u':
        if random.random() < user_invalid_prob:
            data = user_inv_command
        else:
            data = random.randint(*user_good_command)
        print 'Driving u', data
        upub.publish(data)
    elif type == 'c':
        data = random.randint(*crossing_signal)
        print 'Driving c', data
        cpub.publish(data)
    elif type == 'g':
        if random.random() < gaze_bad_prob:
            data = random.uniform(*gaze_bad)
        else:
            data = random.uniform(*gaze_good)
        print 'Driving g', data
        gpub.publish(data)

def runtest():
    no_stimuli = random.randint(*stim_count)
    print 'Running CPRG test,', no_stimuli, 'stimuli'
    TYPES = ('u','c','g')
    std_seq_pos = 3
    for i in range(no_stimuli):
        if std_seq_pos >= len(TYPES) and random.random() < std_seq_bias:
            print 'StdSeq'
            std_seq_pos = 0

        if std_seq_pos < len(TYPES):
            type = TYPES[std_seq_pos]
            std_seq_pos += 1
        else:
            type = random.choice(TYPES)
        make_stimuli(type)
        delay()

if __name__ == '__main__':
    rospy.init_node('cprg_tests')
    rospy.sleep(1)
    set_constraints()
    runtest()
    rospy.sleep(8)
    upub.publish(127) # Shutdown signal
    rospy.sleep(0.5)
    upub.publish(127) # Please
    rospy.sleep(0.1)
