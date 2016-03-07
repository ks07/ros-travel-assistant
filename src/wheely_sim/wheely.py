#!/usr/local/bin/coverage run

import rospy
import smach
import smach_ros
import actionlib
import std_msgs.msg

from wheely_sim.msg import CrossRoadAction, CrossRoadGoal

from coverage import Coverage

def callback_smach(data,state):
    rospy.loginfo('Received: ' + str(data) + ' for ' + type(state).__name__)
    state.sub_callback(data)

class Waiting(smach.State):
    """ The state where wheely is idle on the pavement. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wait','signalwait','shutdown','preempted'],
                             output_keys=['wait_dest_out','wait_midcross_out'])
        # Any state init here
        self.command = -1
        self.sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, callback_smach, self)

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'preempted'
        # State execution here
        rospy.loginfo('Executing state WAITING')

        userdata.wait_midcross_out = False # An annoying feature of smach
        rospy.sleep(0.2)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.command == 127:
            # Quit command, to end a test gracefully.
            return 'wait'
        elif self.command >= 0:
            userdata.wait_dest_out = self.command
            self.command = -1
            return 'signalwait'
        else:
            return 'wait'

    def sub_callback(self, data):
        self.command = data.data

class SignalWaiting(smach.State):
    """ The state where wheely is waiting for the lights on the crossing to turn green. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['signalwait','timeout','cross','preempted'],
                             input_keys=['sigwait_dest_in','sigwait_midcross_in'],
                             output_keys=['sigwait_dest_in'])
        self.ready = False
        self.pub = rospy.Publisher('light_commands', std_msgs.msg.Int8, queue_size=10)
        self.sub = rospy.Subscriber('crossing_signals', std_msgs.msg.Int8, callback_smach, self)

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'preempted'
        rospy.loginfo('Executing state SIGNALWAITING')


        if userdata.sigwait_midcross_in:
            return 'cross'
        else:
            self.pub.publish(data = 1)
            rospy.sleep(0.2)

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

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

class BeginCrossing(smach.State):
    """ The state where we tell the base to start the move op. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted'],
                             input_keys=['begincross_dest_in'],
                             output_keys=['begincross_actcli_out'])

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'preempted'
        rospy.loginfo('Executing state BEGINCROSSING dest: ' + str(userdata.begincross_dest_in))

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Try to interface with actionlib manually
        client = actionlib.SimpleActionClient('cross_road', CrossRoadAction)
        client.wait_for_server()

        goal = CrossRoadGoal()
        goal.crossing_id = userdata.begincross_dest_in
        rospy.loginfo('Asking base to drive to ' + str(goal.crossing_id))
        client.send_goal(goal)
        userdata.begincross_actcli_out = client

        return 'succeeded'

class Crossing(smach.State):
    """ The state where wheel is moving across the road. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted','replanned'],
                             input_keys=['cross_actcli_in'],
                             output_keys=['cross_dest_out','cross_actcli_in','cross_midcross_out']) # Need to mark as output so that the object is mutable
        self.command = -1
        self.sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, callback_smach, self)

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'preempted'
        rospy.loginfo('Executing state CROSSING')

        # Clear out any commands we should have already acted on.
        self.command = -1

        client = userdata.cross_actcli_in
        userdata.cross_midcross_out = True

        TIMEOUT = 45.0
        TIMESTEP = 0.25
        
        for i in range(int(TIMEOUT/TIMESTEP)):
            if self.preempt_requested():
                client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.command != -1:
                client.cancel_goal()
                userdata.cross_dest_out = self.command
                return 'replanned'

            finished = client.wait_for_result(rospy.Duration.from_sec(TIMESTEP))
            if finished:
                userdata.cross_midcross_out = False
                break
        res = client.get_result()
        rospy.loginfo(res)

        return 'succeeded'

    def sub_callback(self, data):
        self.command = data.data

def monitor_commands_cb(ud,msg):
    # ud is a smach.user_data.Remapper instance
    # msg is the message struct
    print ud,msg
    return msg.data != 127

def concurrence_cb(states):
    # Return True to preempt all other states, False to wait, or a list of labels to preempt
    #rospy.signal_shutdown('Monitor shutdown')
    return True

def main():
    rospy.init_node('wheely_node')

    sm_con = smach.Concurrence(outcomes=['continue','done'],
                               default_outcome='continue',
                               child_termination_cb=concurrence_cb,
                               outcome_map={'done':{'MONITOR_COMMANDS':'invalid'}})

    with sm_con:
        # Create a monitor state that will run concurrently
        smach.Concurrence.add('MONITOR_COMMANDS',
                              smach_ros.MonitorState('/user_commands',
                                                     std_msgs.msg.Int8,
                                                     monitor_commands_cb))

        # Create the main state machine
        sm = smach.StateMachine(outcomes=['succeeded','failed'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('WAITING', Waiting(),
                                   transitions={'wait':'WAITING',
                                                'signalwait':'SIGNALWAITING',
                                                'shutdown':'succeeded',
                                                'preempted':'succeeded'},
                                   remapping={'wait_dest_out':'user_dest',
                                              'wait_midcross_out':'is_midcross'})
            smach.StateMachine.add('SIGNALWAITING', SignalWaiting(),
                                   transitions={'signalwait':'SIGNALWAITING',
                                                'cross':'BEGINCROSSING',
                                                'timeout':'WAITING',
                                                'preempted':'WAITING'},
                                   remapping={'sigwait_dest_in':'user_dest',
                                              'sigwait_midcross_in':'is_midcross'})
            smach.StateMachine.add('BEGINCROSSING', BeginCrossing(),
                                   transitions={'succeeded':'CROSSING',
                                                'preempted':'WAITING'},
                                   remapping={'begincross_dest_in':'user_dest',
                                              'begincross_actcli_out':'actcli'})
            smach.StateMachine.add('CROSSING', Crossing(),
                                   transitions={'succeeded':'WAITING',
                                                'preempted':'WAITING',
                                                'replanned':'BEGINCROSSING'},
                                   remapping={'cross_actcli_in':'actcli',
                                              'cross_dest_out':'user_dest',
                                              'cross_midcross_out':'is_midcross'})
                                                

        smach.Concurrence.add('MAIN', sm)

    # Execute smach plan!
    outcome = sm_con.execute()

if __name__ == '__main__':
    try:
        main()
    except smach.exceptions.InvalidUserCodeError as ex:
        if rospy.is_shutdown():
            print 'Shutting down'
        else: # pragma: no cover
            raise
