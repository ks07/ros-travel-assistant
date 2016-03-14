#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Int8

def setupPubs():
    pubs = {
        'user_commands': rospy.Publisher('user_commands',Int8,queue_size=10),
        'crossing_signals': rospy.Publisher('crossing_signals',Int8,queue_size=1)
    }
    return pubs;

def sub_callback(data,args):
    print 'Got ',data,args
    sub_rcvd,tag = args
    sub_rcvd[tag] = data.data

def setupSubs():
    sub_rcvd = {}
    rospy.Subscriber('light_commands',Int8,sub_callback,(sub_rcvd,'light_commands'))
    return sub_rcvd

def main(bdi_test_file):
    rospy.init_node('bdi_interface', anonymous=True)

    pubs = setupPubs()
    sub_rcvd = setupSubs()
    rospy.sleep(0.5) # Need to sleep to allow connections to establish

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
            rospy.sleep(delta)
            if args[0] == 'pub':
                # Publish a2 by a1
                print 'Publishing ',args[2],' to ',args[1]
                pubs[args[1]].publish(data = int(args[2]))
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
    print 'Test finished.'

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: bdi_interface.py <bdi_test_file>")
    bdi_test_file = sys.argv[1]
    main(bdi_test_file)
