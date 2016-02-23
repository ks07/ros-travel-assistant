#!/usr/bin/env python

import roslib
roslib.load_manifest('wheely_sim')
import rospy
import smach
import smach_ros
import actionlib

from wheely_sim.msg import CrossRoadAction, CrossRoadGoal

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
        
        # Try to interface with actionlib manually
        client = actionlib.SimpleActionClient('cross_road', CrossRoadAction)
        client.wait_for_server()

        goal = CrossRoadGoal()
        goal.crossing_id = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(90.0))
        res = client.get_result()
        rospy.loginfo(res)

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
    rospy.spin()

if __name__ == '__main__':
    main()
