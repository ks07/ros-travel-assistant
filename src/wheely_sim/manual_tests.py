#!/usr/bin/env python

# This script runs directed tests, to compare and contrast with BDI

import sys
import rospy
from std_msgs.msg import Int8, Float32

upub = rospy.Publisher('user_commands', Int8, queue_size=1)
cpub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
gpub = rospy.Publisher('gaze_sensor', Float32, queue_size=1)

def runtest(testno):
    # Easy way to call these functions without writing out a map or a huge if
    fname = 't' + str(testno)
    testfunc = globals()[fname]
    return testfunc()

def t1():
    # Normal operation
    upub.publish(1)
    cpub.publish(1)
    gpub.publish(1.0)

def t2():
    # No gaze
    upub.publish(1)
    cpub.publish(1)
    gpub.publish(0.0)

def t3():
    # No crossing
    upub.publish(1)
    cpub.publish(0)
    gpub.publish(1.0)

def t4():
    # Redirect while crossing
    t1()
    rospy.sleep(0.1)
    upub.publish(0)

def mantest():
    utype = sys.argv[1]
    udata = sys.argv[2]


    rospy.sleep(0.2)
    pub.publish(data = udata)
    rospy.sleep(0.1)


if len(sys.argv) < 2:
    print "Need to specify a test number."
    sys.exit(1)
else:
    rospy.init_node('manual_tests')
    rospy.sleep(1)
    testno = int(sys.argv[1])
    runtest(testno)
    rospy.sleep(8)
    upub.publish(127) # Shutdown signal
    rospy.sleep(0.1)
