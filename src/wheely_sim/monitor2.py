#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import itertools
import os
from std_msgs.msg import Int8, Float32
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
    # TODO: Road bounds need to be parameterised, really
    areas = rospy.get_param('/wheely_sim/road_areas')
    for top,bot in grouper(areas, 2, -3.0):
        if position.y < top and position.y > bot:
            return True
    return False

class TrafficTrigger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['triggered','inactive','triggerwait','shutdown'])
        self.trigger = threading.Event()
        # Assume lights start red, so should trigger immediately
        self.nogaze = True
        self.red = True
        self.trigger.set()
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.gaze_sub = rospy.Subscriber('gaze_sensor', Float32, self.gaze_cb)
        self.onroad = False

    def light_cb(self, msg):
        self.red = msg.data == 0
        if self.red or self.nogaze:
            # Crossing is red, we should now move to checking for movement
            self.trigger.set()
        else:
            self.trigger.clear()

    def odom_cb(self, msg):
        self.onroad = within_road(msg.pose.pose.position)

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.trigger.set()

    def gaze_cb(self, msg):
        self.nogaze = msg.data < 0.8
        if self.nogaze or self.red:
            self.trigger.set()
        else:
            self.trigger.clear()

    def execute(self, userdata):
        rospy.loginfo('Waiting for red lights.')
        triggered = self.trigger.wait(10000)
        if testend:
            return 'shutdown'

        if triggered:
            rospy.loginfo('Lights became red.')
            if self.onroad:
                return 'triggerwait'
            else:
                log('t',False)
                return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class LocationChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','failed','shutdown'])
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.gaze_sub = rospy.Subscriber('gaze_sensor', Float32, self.gaze_cb)
        self.change = threading.Event()
        self.gaze = False
        self.green = False
        self.reset = False
        self.onroad = False
    
    def light_cb(self, msg):
        self.green = msg.data == 1
        if self.green and self.gaze:
            # Crossing has gone green and gaze back, reset the monitor
            self.reset = True
            self.change.set()
        else:
            self.change.clear()
    
    def odom_cb(self, msg):
        self.onroad = within_road(msg.pose.pose.position)
        if self.onroad:
            self.change.set()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.change.set()

    def gaze_cb(self, msg):
        self.gaze = msg.data >= 0.8
        if self.gaze and self.green:
            # Crossing has gone green and gaze back, reset the monitor
            self.reset = True
            self.change.set()
        else:
            self.change.clear()
        
    def execute(self, userdata):
        self.change.clear() # Need to reset the event
        rospy.loginfo('Crossing red, watching for movement.')
        self.change.wait()

        if testend:
            return 'shutdown'
        elif self.reset:
            rospy.loginfo('RESET')
            log(1)
            return 'inactive'
        elif self.onroad:
            rospy.loginfo('FAIL')
            log(0)
            return 'failed'
        else:
            rospy.logerr('No condition set when unblocked!')
            return 'inactive'

class RoadWait(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','cleared','shutdown'])
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.change = threading.Event()
        self.reset = False
        self.onroad = False
    
    def light_cb(self, msg):
        if msg.data == 1:
            # Crossing has gone green, reset the monitor
            self.reset = True
            self.change.set()
        else:
            self.reset = False
    
    def odom_cb(self, msg):
        self.onroad = within_road(msg.pose.pose.position)
        if not self.onroad:
            self.change.set()

    def ucmd_cb(self, msg):
        global testend
        if msg.data == 127:
            testend = True
            self.change.set()
        
    def execute(self, userdata):
        self.change.clear() # Need to reset the event
        rospy.loginfo('Waiting until road cleared.')
        self.change.wait()

        if testend:
            return 'shutdown'
        elif self.reset:
            rospy.loginfo('RESET')
            return 'inactive'
        elif not self.onroad:
            rospy.loginfo('CLEARED')
            log('w',False)
            return 'cleared'
        else:
            rospy.logerr('No condition set when unblocked!')
            return 'inactive'

def main():
	rospy.init_node('assertion2', anonymous=True)

    	sm = smach.StateMachine(outcomes=['done'])

   	with sm:
		smach.StateMachine.add('TRAFFICTRIGGER', TrafficTrigger(), 
                                       transitions={'triggered':'LOCATIONCHECKER',
                                                    'triggerwait':'ROADWAIT',
                                                    'inactive':'TRAFFICTRIGGER',
                                                    'shutdown':'done'})
		smach.StateMachine.add('LOCATIONCHECKER', LocationChecker(),
                                       transitions={'inactive':'TRAFFICTRIGGER',
                                                    'failed':'ROADWAIT',
                                                    'shutdown':'done'})
                smach.StateMachine.add('ROADWAIT', RoadWait(),
                                       transitions={'cleared':'LOCATIONCHECKER',
                                                    'inactive':'TRAFFICTRIGGER',
                                                    'shutdown':'done'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
    if 'MONITORLOG' in os.environ:
        logfile = os.environ['MONITORLOG']
    else:
        logfile = '.monitor2'

    with open(logfile, 'w') as f:
        try:
            main()
        except	rospy.ROSInterruptException:
            pass
