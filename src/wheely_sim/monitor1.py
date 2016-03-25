#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import time
from std_msgs.msg import Empty,Int8

class CommandTrigger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['triggered','inactive'],
                             output_keys=['ct_time_out'])
        self.trigger = threading.Event()
        self.ucmd_sub = rospy.Subscriber('user_commands', Int8, self.ucmd_cb)
        self.ttime = 0.0

    def ucmd_cb(self, msg):
        self.ttime = time.time()
        self.trigger.set()

    def execute(self, userdata):
        rospy.loginfo('Waiting for command.')
        triggered = self.trigger.wait(10000)
        if triggered:
            userdata.ct_time_out = self.ttime
            self.trigger.clear()
            return 'triggered'
        else:
            # Add a timeout so that the monitor can be shut off.
            return 'inactive'

class ResponseChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['passed','failed'],
                             input_keys=['rc_time_in'])
        self.tm1_sub = rospy.Subscriber('test_monitor1', Empty, self.tm1_cb)
        self.responded = threading.Event()
    
    def tm1_cb(self, msg):
        self.responded.set()
        
    def execute(self, userdata):
        self.responded.clear() # Need to reset the event
        rospy.loginfo('On road, watching for stops.')
        self.responded.wait()

        now = time.time()
        
        if time.time() - userdata.rc_time_in < 15.0:
            rospy.loginfo('PASS')
            return 'passed'
        else:
            rospy.loginfo('FAIL')
            return 'failed'

def main():
	rospy.init_node('assertion1', anonymous=True)

    	sm = smach.StateMachine(outcomes=['done'])

   	with sm:
		smach.StateMachine.add('COMMANDTRIGGER', CommandTrigger(), 
                                       transitions={'triggered':'RESPONSECHECKER',
                                                    'inactive':'COMMANDTRIGGER'},
                                       remapping={'ct_time_out':'time'})
		smach.StateMachine.add('RESPONSECHECKER', ResponseChecker(),
                                       transitions={'passed':'COMMANDTRIGGER',
                                                    'failed':'COMMANDTRIGGER'},
                                       remapping={'rc_time_in':'time'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main()
	except	rospy.ROSInterruptException:
		pass
