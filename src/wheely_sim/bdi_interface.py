#!/usr/bin/env python

import rospy
import sys
import threading
import random
from std_msgs.msg import Int8,Empty,Float32

def tsub_callback(msg):
    global start_trigger
    start_trigger.set()

def setupPubs():
    pubs = {
        'user_commands': (rospy.Publisher('user_commands',Int8,queue_size=10),int),
        'crossing_signals': (rospy.Publisher('crossing_signals',Int8,queue_size=1),int),
        'gaze_sensor': (rospy.Publisher('gaze_sensor',Float32,queue_size=1),float)
    }
    return pubs;

def sub_callback(data,args):
    rospy.logdebug('Got ' + str(data) + str(args))
    sub_rcvd,tag = args
    sub_rcvd[tag] = data.data

def setupSubs():
    sub_rcvd = {}
    rospy.Subscriber('light_commands',Int8,sub_callback,(sub_rcvd,'light_commands'))
    return sub_rcvd

IGNORES_DELTA = ['waitfor','clear','paramdelay']
PARAM_PFX = '/bdiparam/'

def main(bdi_test_file, trigger):
    rospy.init_node('bdi_interface', anonymous=True)

    pubs = setupPubs()
    sub_rcvd = setupSubs()

    if trigger:
        tpub = rospy.Publisher('test_trigger',Empty,queue_size=1)
        rospy.sleep(0.5) # Need to sleep to allow connections to establish
        block = raw_input('Hit enter when all test drivers ready...')
        tpub.publish()
    else:
        global start_trigger
        start_trigger = threading.Event()
        tsub = rospy.Subscriber('test_trigger',Empty,tsub_callback)
        print 'Waiting for start signal...'
        start_trigger.wait()

    with open(bdi_test_file, "r") as f:
        begin = True
        ts = 0
        for line in f:
            args = line.strip().split(',')
            prev_ts = ts
            ts = float(args.pop(0))
            if begin:
                prev_ts = ts
                begin = False
            delta = (ts - prev_ts) / 1000
            print delta,args
            if args[0] not in IGNORES_DELTA:
                rospy.sleep(delta)
            if args[0] == 'pub':
                # Publish a2 by a1
                print 'Publishing ',args[2],' to ',args[1]
                pset = pubs[args[1]]
                rdata = 127 if args[2] == '127' else get_param(args[1])
                pset[0].publish(data = pset[1](rdata))
                rospy.sleep(0.5)
            elif args[0] == 'waitfor':
                # Wait until we see a2 from a1
                print 'Waiting for ',args[2],' from ',args[1]
                while args[1] not in sub_rcvd or sub_rcvd[args[1]] != int(args[2]):
                    rospy.sleep(0.5) # This is probably bad
            elif args[0] == 'clear':
                # Clears the last seen signal from a1
                print 'Cleared ',args[1]
                del sub_rcvd[args[1]]
            elif args[0] == 'paramdelay':
                # Waits for some amount of time, parameterised
                wait_time = get_param(args[1])
                wait_time = wait_time / 1000
                print 'Waiting at ',args[1],' for ',wait_time
                rospy.sleep(wait_time)
    print 'Test finished.'

def get_param(key):
    bounds = rospy.get_param(PARAM_PFX + key)
    v = random.uniform(*bounds)
    return v

if __name__ == "__main__":
    if len(sys.argv) not in [2,3]:
        sys.exit("Usage: bdi_interface.py <bdi_test_file> [-t]")
    bdi_test_file = sys.argv[1]
    trigger = (len(sys.argv) == 3)
    main(bdi_test_file, trigger)
