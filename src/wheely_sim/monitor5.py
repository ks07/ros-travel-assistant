#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import itertools
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

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

class RoadTrigger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['triggered','inactive','shutdown'])
        self.trigger = threading.Event()
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.onroad = False

    def odom_cb(self, msg):
        if within_road(msg.pose.pose.position):
            self.trigger.set()
        else:
            self.trigger.clear()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.trigger.set()

    def execute(self, userdata):
        rospy.loginfo('Waiting for road entry.')
        triggered = self.trigger.wait(10000)
        if testend:
            return 'shutdown'
        elif triggered:
            log('t',False)
            return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class StopChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','failed','shutdown'])
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.change = threading.Event()
        self.onroad = False
        self.moving = False
    
    def odom_cb(self, msg):
        self.moving = msg.twist.twist.linear.y != 0
        self.onroad = within_road(msg.pose.pose.position)
        if not self.onroad or not self.moving:
            self.change.set()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.change.set()
        
    def execute(self, userdata):
        if testend:
            return 'shutdown'
        self.change.clear() # Need to reset the event
        rospy.loginfo('On road, watching for stops.')
        self.change.wait()

        if testend:
            return 'shutdown'
        elif not self.onroad:
            log(1)
            rospy.loginfo('RESET')
            return 'inactive'
        else:
            log(0)
            rospy.loginfo('FAIL')
            return 'failed'

def main():
	rospy.init_node('assertion5', anonymous=True)

    	sm = smach.StateMachine(outcomes=['done'])

   	with sm:
		smach.StateMachine.add('ROADTRIGGER', RoadTrigger(), 
                                       transitions={'triggered':'STOPCHECKER',
                                                    'inactive':'ROADTRIGGER',
                                                    'shutdown':'done'})
		smach.StateMachine.add('STOPCHECKER', StopChecker(),
                                       transitions={'inactive':'ROADTRIGGER',
                                                    'failed':'ROADTRIGGER',
                                                    'shutdown':'done'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
    if 'MONITORLOG' in os.environ:
        logfile = os.environ['MONITORLOG']
    else:
        logfile = '.monitor5'

    with open(logfile, 'w') as f:
	try:
		main()
	except	rospy.ROSInterruptException:
		pass
