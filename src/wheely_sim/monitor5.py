#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import itertools
from nav_msgs.msg import Odometry

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
                             outcomes=['triggered','inactive'])
        self.trigger = threading.Event()
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.onroad = False

    def odom_cb(self, msg):
        if within_road(msg.pose.pose.position):
            self.trigger.set()
        else:
            self.trigger.clear()

    def execute(self, userdata):
        rospy.loginfo('Waiting for road entry.')
        triggered = self.trigger.wait(10000)
        if triggered:
            return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class StopChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inactive','failed'])
        self.odom_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odom_cb)
        self.change = threading.Event()
        self.onroad = False
        self.moving = False
    
    def odom_cb(self, msg):
        self.moving = msg.twist.twist.linear.y != 0
        self.onroad = within_road(msg.pose.pose.position)
        if not self.onroad or not self.moving:
            self.change.set()
        
    def execute(self, userdata):
        self.change.clear() # Need to reset the event
        rospy.loginfo('On road, watching for stops.')
        self.change.wait()

        if not self.onroad:
            rospy.loginfo('RESET')
            return 'inactive'
        else:
            rospy.loginfo('FAIL')
            return 'failed'

def main():
	rospy.init_node('assertion5', anonymous=True)

    	sm = smach.StateMachine(outcomes=['done'])

   	with sm:
		smach.StateMachine.add('ROADTRIGGER', RoadTrigger(), 
                                       transitions={'triggered':'STOPCHECKER',
                                                    'inactive':'ROADTRIGGER'})
		smach.StateMachine.add('STOPCHECKER', StopChecker(),
                                       transitions={'inactive':'ROADTRIGGER',
                                                    'failed':'ROADTRIGGER'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main()
	except	rospy.ROSInterruptException:
		pass
