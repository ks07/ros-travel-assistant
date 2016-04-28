#!/usr/bin/env python

# This script runs directed tests, to compare and contrast with BDI

import sys
import rospy
import threading
from std_msgs.msg import Int8, Float32

upub = rospy.Publisher('user_commands', Int8, queue_size=1)
cpub = rospy.Publisher('crossing_signals', Int8, queue_size=1)
gpub = rospy.Publisher('gaze_sensor', Float32, queue_size=1)

eshutdown = threading.Event()

def runtest(testno):
    # Easy way to call these functions without writing out a map or a huge if
    fname = 't_' + str(testno)
    print 'Launching ',fname
    testfunc = globals()[fname]
    return testfunc()

def t_0():
    # Normal operation
    upub.publish(1)
    cpub.publish(1)
    gpub.publish(1.0)

def t_1():
    # Current location
    upub.publish(0)
    cpub.publish(1)
    gpub.publish(1.0)

def t_2():
    # No gaze
    upub.publish(1)
    cpub.publish(1)
    gpub.publish(0.0)

def t_3():
    # No crossing
    upub.publish(1)
    cpub.publish(0)
    gpub.publish(1.0)

def t_4():
    # Redirect while crossing
    t_0()
    rospy.sleep(1.0)
    upub.publish(0)

def t_5():
    # Repeat command while crossing
    t_0()
    rospy.sleep(0.1)
    upub.publish(1)

def t_6():
    # Go and come back, realistic env
    t_0()
    rospy.sleep(1)
    gpub.publish(0.6)
    rospy.sleep(6)
    cpub.publish(0)
    rospy.sleep(0.5)
    upub.publish(0)
    gpub.publish(0.9)
    rospy.sleep(0.5)
    cpub.publish(1)

def t_7():
    # Interrupt cross, early
    t_0()
    rospy.sleep(0.1)
    cpub.publish(0)

def t_8():
    # Interrupt cross, island, continue
    t_0()
    rospy.sleep(2)
    cpub.publish(0)
    rospy.sleep(1)
    cpub.publish(1)

def t_9():
    # Interrupt cross, island, new command, cont
    t_0()
    rospy.sleep(2)
    cpub.publish(0)
    upub.publish(0)
    rospy.sleep(1)
    cpub.publish(1)

def t_10():
    # Interrupt cross, island, repeat command
    t_0()
    rospy.sleep(2)
    cpub.publish(0)
    upub.publish(1)
    rospy.sleep(1)
    cpub.publish(1)

def t_11():
    # Interrupt cross, island, no gaze
    t_0()
    rospy.sleep(2)
    cpub.publish(0)
    rospy.sleep(0.5)
    gpub.publish(0.3)
    rospy.sleep(1)
    cpub.publish(1)

def t_12():
    # Interrupt cross, try to keep on road.
    t_0()
    rospy.sleep(1.5)
    cpub.publish(0)
    rospy.sleep(0.2)
    upub.publish(1)
    rospy.sleep(0.1)
    upub.publish(0)
    rospy.sleep(0.1)
    upub.publish(2)
    rospy.sleep(0.1)
    upub.publish(1)
    rospy.sleep(0.1)
    upub.publish(0)

def t_13():
    # Road wobble.
    t_0()
    rospy.sleep(4.0)
    t_1()
    rospy.sleep(3.5)
    t_0()
    rospy.sleep(3.5)
    t_1()

def t_14():
    # Invalid dest waiting
    upub.publish(2)
    cpub.publish(1)
    gpub.publish(1.0)

def gazeokpubloop():
    rate = rospy.Rate(100) # Hz
    for i in range(1000): # Need to be very quick to hit begincrossing
        gpub.publish(1.0)
    while not rospy.is_shutdown() and not eshutdown.is_set():
        gpub.publish(1.0)
        rate.sleep()

def t_15():
    # Flood with lots of gaze okay commands whilst crossing
    gthread = threading.Thread(target = gazeokpubloop)
    gthread.start()
    upub.publish(1)
    rospy.sleep(0.5)
    cpub.publish(1)
    rospy.sleep(4)
    cpub.publish(0)

def gazelowpubloop():
    print 'go'
    rospy.sleep(0.1)
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown() and not eshutdown.is_set():
        gpub.publish(0.4)
        rate.sleep()

def t_16():
    # Flood with lots of gaze low commands whilst crossing - need to allow cross
    gthread = threading.Thread(target = gazelowpubloop)
    upub.publish(1)
    gpub.publish(0.9)
    cpub.publish(1)
    #rospy.sleep(0.1)
    gthread.start()
    rospy.sleep(1)
    cpub.publish(0)

def sigredpubloop():
    rate = rospy.Rate(1000) # Hz
    while not rospy.is_shutdown() and not eshutdown.is_set():
        cpub.publish(0)
        rate.sleep()

def t_17():
    # Flood with lots of crossing red commands whilst crossing
    gthread = threading.Thread(target = sigredpubloop)
    gthread.start()
    upub.publish(1)
    rospy.sleep(0.5)
    cpub.publish(1)
    rospy.sleep(4)
    cpub.publish(0)

def siggreenpubloop():
    rate = rospy.Rate(100) # Hz
    for i in range(1000): # Need to be very quick to hit begincrossing
        cpub.publish(1)
    while not rospy.is_shutdown() and not eshutdown.is_set():
        cpub.publish(1)
        rate.sleep()

def t_18():
    # Flood with lots of crossing green commands whilst crossing
    gthread = threading.Thread(target = siggreenpubloop)
    gthread.start()
    upub.publish(1)
    rospy.sleep(0.5)
    gpub.publish(1.0)

def t_19():
    # Repeat user command whilst waiting for confirmation
    upub.publish(1)
    rospy.sleep(0.5)
    cpub.publish(1)
    upub.publish(1)
    rospy.sleep(0.5)
    gpub.publish(1.0)

def t_20():
    # New user command whilst waiting for confirmation
    upub.publish(1)
    rospy.sleep(0.5)
    cpub.publish(1)
    upub.publish(0)
    rospy.sleep(0.5)
    gpub.publish(1.0)

def t_21():
    # Bad user command whilst waiting for confirmation
    upub.publish(1)
    rospy.sleep(0.5)
    cpub.publish(1)
    upub.publish(5)
    rospy.sleep(0.5)
    gpub.publish(1.0)

def t_22():
    # Invalid command on road
    t_0()
    rospy.sleep(1.0)
    upub.publish(2)

if len(sys.argv) < 2:
    print max(int(t[2:]) for t in globals() if t.startswith('t_'))
    sys.exit(1)
else:
    rospy.init_node('manual_tests')
    rospy.sleep(1)
    testno = int(sys.argv[1])
    runtest(testno)
    rospy.sleep(8)
    eshutdown.set()
    rospy.sleep(1)
    upub.publish(127) # Shutdown signal
    rospy.sleep(0.1)
