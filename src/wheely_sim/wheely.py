#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait','cross'])
        # Any state init here
        self.counter = 0

    def execute(self, userdata):
        # State execution here
        rospy.loginfo('Executing state WAITING')
        if self.counter < 3:
            self.counter += 1
            return 'wait'
        else:
            return 'cross'
    
class Crossing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CROSSING')
        return 'succeeded'

def main():
    rospy.init_node('wheely_node')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['succeeded','failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAITING', Waiting(),
                               transitions={'wait':'WAITING',
                                            'cross':'CROSSING'})
        smach.StateMachine.add('CROSSING', Crossing(),
                               transitions={'succeeded':'succeeded'})

    # Execute smach plan!
    outcome = sm.execute()

if __name__ == '__main__':
    main()
