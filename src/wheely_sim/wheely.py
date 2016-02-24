#!/usr/bin/env python

import random

import roslib
roslib.load_manifest('wheely_sim')
import rospy
import smach
import smach_ros
import actionlib
import std_msgs.msg

from wheely_sim.msg import CrossRoadAction, CrossRoadGoal

def callback_smach(data,state):
    rospy.loginfo('Received: ' + str(data) + ' for ' + type(state).__name__)
    state.sub_callback(data)

class Waiting(smach.State):
    """ The state where wheely is idle on the pavement. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wait','signalwait'],
                             output_keys=['wait_dest_out'])
        # Any state init here
        self.command = -1

    def execute(self, userdata):
        # State execution here
        rospy.loginfo('Executing state WAITING')
        rospy.sleep(0.2)
        rospy.Subscriber('user_commands', std_msgs.msg.Int8, callback_smach, self)
        rospy.sleep(0.1)
        if self.command > 0:
            userdata.wait_dest_out = self.command
            self.command = -1
            return 'signalwait'
        else:
            return 'wait'

    def sub_callback(self, data):
        self.command = data

class SignalWaiting(smach.State):
    """ The state where wheely is waiting for the lights on the crossing to turn green. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['signalwait','timeout','cross'],
                             input_keys=['sigwait_dest_in'],
                             output_keys=['sigwait_dest_out'])
        self.ready = False

    def execute(self, userdata):
        rospy.loginfo('Executing state SIGNALWAITING')
        rospy.sleep(0.2)
        rospy.Subscriber('crossing_signals', std_msgs.msg.Int8, callback_smach, self)
        rospy.sleep(0.1)
        if self.ready:
            self.ready = False
            return 'cross'
        else:
            return 'signalwait'

    def sub_callback(self, data):
        if data.data == 99:
            self.ready = True
        else:
            self.ready = False


class Crossing(smach.State):
    """ The state where wheely is moving across the road. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['cross_dest_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CROSSING dest: ' + str(userdata.cross_dest_in))
        
        # Try to interface with actionlib manually
        client = actionlib.SimpleActionClient('cross_road', CrossRoadAction)
        client.wait_for_server()

        goal = CrossRoadGoal()
        goal.crossing_id = bool(userdata.cross_dest_in)
        rospy.loginfo('Asking base to drive to ' + str(goal.crossing_id))
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
                                            'signalwait':'SIGNALWAITING'},
                               remapping={'wait_dest_out':'user_dest'})
        smach.StateMachine.add('SIGNALWAITING', SignalWaiting(),
                               transitions={'signalwait':'SIGNALWAITING',
                                            'cross':'CROSSING',
                                            'timeout':'WAITING'},
                               remapping={'sigwait_dest_in':'user_dest',
                                          'sigwait_dest_out':'user_dest'})
        smach.StateMachine.add('CROSSING', Crossing(),
                               transitions={'succeeded':'WAITING'},
                               remapping={'cross_dest_in':'user_dest'})

    # Execute smach plan!
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
