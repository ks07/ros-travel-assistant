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

def t0():
    # Normal operation
    upub.publish(1)
    cpub.publish(1)
    gpub.publish(1.0)

def t1():
    # Current location
    upub.publish(0)
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

def t5():
    # Repeat command while crossing
    t1()
    rospy.sleep(0.1)
    upub.publish(1)

def t6():
    # Go and come back
    t1()
    rospy.sleep(7)
    upub.publish(0)

def t7():
    # Interrupt cross, early
    t1()
    rospy.sleep(0.1)
    cpub.publish(0)

def t8():
    # Interrupt cross, island, continue
    t1()
    rospy.sleep(3)
    cpub.publish(0)
    rospy.sleep(1)
    cpub.publish(1)

def t9():
    # Interrupt cross, island, new command, cont
    t1()
    rospy.sleep(3)
    cpub.publish(0)
    upub.publish(0)
    rospy.sleep(1)
    cpub.publish(1)

def t10():
    # Interrupt cross, island, repeat command
    t1()
    rospy.sleep(3)
    cpub.publish(0)
    upub.publish(1)
    rospy.sleep(1)
    cpub.publish(1)

def t11():
    # Interrupt cross, island, no gaze
    t1()
    rospy.sleep(3)
    cpub.publish(0)
    rospy.sleep(0.5)
    gpub.publish(0.3)
    rospy.sleep(1)
    cpub.publish(1)

def t12():
    # Interrupt cross, try to keep on road.
    t1()
    rospy.sleep(1.5)
    cpub.publish(0)
    rospy.sleep(0.1)
    upub.publish(1)
    upub.publish(0)
    upub.publish(1)
    upub.publish(0)

def t13():
    # Road wobble.
    t0()
    rospy.sleep(4.0)
    t1()
    rospy.sleep(3.5)
    t0()
    rospy.sleep(3.5)
    t1()

if len(sys.argv) < 2:
    print max(int(t[1:]) for t in globals() if t.startswith('t'))
    sys.exit(1)
else:
    rospy.init_node('manual_tests')
    rospy.sleep(1)
    testno = int(sys.argv[1])
    runtest(testno)
    rospy.sleep(8)
    upub.publish(127) # Shutdown signal
    rospy.sleep(0.1)
