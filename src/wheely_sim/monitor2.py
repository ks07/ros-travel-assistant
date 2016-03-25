#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import itertools
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry

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
                             outcomes=['triggered','inactive','triggerwait'])
        self.trigger = threading.Event()
        # Assume lights start red, so should trigger immediately
        self.trigger.set()
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.onroad = False

    def light_cb(self, msg):
        if msg.data == 0:
            # Crossing is red, we should now move to checking for movement
            self.trigger.set()
        else:
            self.trigger.clear()

    def odom_cb(self, msg):
        self.onroad = within_road(msg.pose.pose.position)

    def execute(self, userdata):
        rospy.loginfo('Waiting for red lights.')
        triggered = self.trigger.wait(10000)
        if triggered:
            rospy.loginfo('Lights became red.')
            if self.onroad:
                return 'triggerwait'
            else:
                return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class LocationChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','failed'])
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
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
        if self.onroad:
            self.change.set()
        
    def execute(self, userdata):
        self.change.clear() # Need to reset the event
        rospy.loginfo('Crossing red, watching for movement.')
        self.change.wait()

        if self.reset:
            rospy.loginfo('RESET')
            return 'inactive'
        elif self.onroad:
            rospy.loginfo('FAIL')
            return 'failed'
        else:
            rospy.logerr('No condition set when unblocked!')
            return 'inactive'

class RoadWait(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','cleared'])
        self.light_sub = rospy.Subscriber('crossing_signals', Int8, self.light_cb)
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
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
        
    def execute(self, userdata):
        self.change.clear() # Need to reset the event
        rospy.loginfo('Waiting until road cleared.')
        self.change.wait()

        if self.reset:
            rospy.loginfo('RESET')
            return 'inactive'
        elif not self.onroad:
            rospy.loginfo('CLEARED')
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
                                                    'inactive':'TRAFFICTRIGGER'})
		smach.StateMachine.add('LOCATIONCHECKER', LocationChecker(),
                                       transitions={'inactive':'TRAFFICTRIGGER',
                                                    'failed':'ROADWAIT'})
                smach.StateMachine.add('ROADWAIT', RoadWait(),
                                       transitions={'cleared':'LOCATIONCHECKER',
                                                    'inactive':'TRAFFICTRIGGER'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main()
	except	rospy.ROSInterruptException:
		pass
