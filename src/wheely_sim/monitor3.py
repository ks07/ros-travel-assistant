#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import itertools
import os
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry

testend = False

def log(msg,term = True):
    global f
    f.write(str(msg))
    if term:
        f.write('\n')
    else:
        f.write(',')

# From itertools recipes
def grouper(iterable, n, fillvalue=None):
    "Collect data into fixed-length chunks or blocks"
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx
    args = [iter(iterable)] * n
    return itertools.izip_longest(fillvalue=fillvalue, *args)

def within_road(position):
    areas = rospy.get_param('/wheely_sim/road_areas')
    for top,bot in grouper(areas, 2, -3.0):
        if position.y < top and position.y > bot:
            return True
    return False

def nearest_refuges(position):
    areas = rospy.get_param('/wheely_sim/road_areas')
    map_bounds = rospy.get_param('/wheely_sim/map_bounds')
    eareas = areas + map_bounds
    for i in range(0,len(areas),2):
        top = eareas[i]
        bot = eareas[i+1]
        if position.y < top and position.y > bot:
            # Found the current interval, pick out the preceeding and following gaps.
            prevint = (eareas[i-1], top)
            nextint = (bot, eareas[i+2])
            return (prevint, nextint)
    rospy.logerr('Refuge search failed!')
    return False

class InterruptTrigger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['triggered','inactive','shutdown'],
                             output_keys=['it_location_out'])
        self.trigger = threading.Event()
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.onroad = False
        self.red = True
        self.location = None

    def light_cb(self, msg):
        self.red = msg.data == 0
        if self.red and self.onroad:
            # Both conditions met, go!
            self.trigger.set()
        else:
            self.trigger.clear()

    def odom_cb(self, msg):
        self.location = msg.pose.pose.position
        self.onroad = within_road(msg.pose.pose.position)
        if self.onroad and self.red:
            self.trigger.set()
        else:
            self.trigger.clear()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.trigger.set()

    def execute(self, userdata):
        rospy.loginfo('Waiting for red lights whilst crossing.')
        triggered = self.trigger.wait(10000)
        if testend:
            return 'shutdown'
        elif triggered:
            log(self.location.y,False)
            userdata.it_location_out = self.location
            return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class RefugeChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','passed','failed','shutdown'],
                             input_keys=['rc_location_in'])
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.change = threading.Event()
        self.green = False
        self.refuged = False
        self.pref = None
    
    def light_cb(self, msg):
        self.green = msg.data == 1
        self.change.set()

    def odom_cb(self, msg):
        # Need to check off road -and- stopped
        if msg.twist.twist.linear.y == 0.0 and not within_road(msg.pose.pose.position):
            # check if we're in preferred
            if self.pref:
                y = msg.pose.pose.position.y
                self.refuged = y < self.pref[0] and y > self.pref[1]
                self.change.set()
            else:
                self.refuged = True
                self.change.set()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.change.set()
        
    def execute(self, userdata):
        if testend:
            return 'shutdown'
        self.change.clear()
        self.refuged = False

        # Find the safe space the robot should move to.
        spaces = nearest_refuges(userdata.rc_location_in)

        y = userdata.rc_location_in.y
        gap = abs(spaces[0][1] - spaces[1][0])
        d1 = abs(y - spaces[0][1]) / gap
        d2 = abs(y - spaces[1][0]) / gap
        
        if d1 < 0.3:
            # Should go here
            self.pref = spaces[0]
        elif d2 < 0.3:
            # Should go here
            self.pref = spaces[1]
        else:
            # Go to either
            self.pref = None

        self.change.wait()

        if testend:
            return 'shutdown'
        elif self.green:
            rospy.loginfo('RESET')
            return 'inactive'
        elif self.refuged:
            log(1)
            rospy.loginfo('PASSED')
            return 'passed'
        else:
            log(0)
            rospy.loginfo('FAIL')
            return 'failed'


def main():
	rospy.init_node('assertion3', anonymous=True)

    	sm = smach.StateMachine(outcomes=['done'])

   	with sm:
		smach.StateMachine.add('INTERRUPTTRIGGER', InterruptTrigger(), 
                                       transitions={'triggered':'REFUGECHECKER',
                                                    'inactive':'INTERRUPTTRIGGER',
                                                    'shutdown':'done'},
                                       remapping={'it_location_out':'intloc'})
		smach.StateMachine.add('REFUGECHECKER', RefugeChecker(),
                                       transitions={'inactive':'INTERRUPTTRIGGER',
                                                    'passed':'INTERRUPTTRIGGER',
                                                    'failed':'INTERRUPTTRIGGER',
                                                    'shutdown':'done'},
                                       remapping={'rc_location_in':'intloc'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
    if 'MONITORLOG' in os.environ:
        logfile = os.environ['MONITORLOG']
    else:
        logfile = '.monitor3'

    with open(logfile, 'w') as f:
	try:
		main()
	except	rospy.ROSInterruptException:
		pass
