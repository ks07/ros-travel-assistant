#!/usr/local/bin/coverage run

import rospy
import smach
import smach_ros
import actionlib
import threading
import time
import std_msgs.msg

from wheely_sim.msg import CrossRoadAction, CrossRoadGoal

from itertools import tee, izip
def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

class SubscriberContainer(object):
    """ Holder for subscribers so they can be shared amongst states. """

    def __init__(self):
        self.uc_data = -1
        self.uc_sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, self.uc_callback)

        self.cs_ready = -1
        self.cs_sub = rospy.Subscriber('crossing_signals', std_msgs.msg.Int8, self.cs_callback)

    def uc_callback(self,msg):
        self.uc_data = msg.data

    def cs_callback(self,msg):
        self.cs_ready = msg.data == 1

class Waiting(smach.State):
    """ The state where wheely is idle on the pavement. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wait','signalwait','shutdown','preempted'],
                             output_keys=['wait_dest_out','wait_midcross_out'])
        # Any state init here
        self.command = -1
        self.sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, self.sub_callback)

    def execute(self, userdata):
        if rospy.is_shutdown(): # pragma: no cover
            return 'preempted'
        # State execution here
        rospy.loginfo('Executing state WAITING')

        userdata.wait_midcross_out = -1 # An annoying feature of smach
        rospy.sleep(0.2)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.command == 127:
            # Quit command, to end a test gracefully.
            return 'shutdown'
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
                             outcomes=['timeout','cross','preempted'],
                             input_keys=['sigwait_dest_in','sigwait_midcross_in'],
                             output_keys=['sigwait_dest_in'])
        self.ready = False
        self.gaze = False
        self.pub = rospy.Publisher('light_commands', std_msgs.msg.Int8, queue_size=10)
        self.sub = rospy.Subscriber('crossing_signals', std_msgs.msg.Int8, self.sub_callback)
        self.uasub = rospy.Subscriber('gaze_sensor', std_msgs.msg.Float32, self.uasub_callback)
        self.ucsub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, self.ucsub_callback)
        self.trigger = threading.Event()

        self.prev_dest = 0 # To recall previous destination
        self.new_dest = -1

        self.tm1_pub = rospy.Publisher('test_monitor1', std_msgs.msg.Empty, queue_size=1)

    def execute(self, userdata):
        global subs

        self.new_dest = -1

        if rospy.is_shutdown(): # pragma: no cover
            return 'preempted'
        rospy.loginfo('Executing state SIGNALWAITING')

        if userdata.sigwait_midcross_in < 0:
            self.prev_dest = userdata.sigwait_dest_in

            self.pub.publish(data = 1)
            
            # Timeout after 30 seconds
            success = self.trigger.wait(25.0)

            if self.new_dest != -1:
                userdata.sigwait_dest_in = self.new_dest

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if success:
                # Assertion monitor
                self.tm1_pub.publish()

                return 'cross'
            else:
                # Assertion monitor
                self.tm1_pub.publish()

                return 'timeout'
        else:
            return 'cross'

    def sub_callback(self, data):
        self.ready = data.data == 1
        if self.ready and self.gaze:
            self.trigger.set()
        else:
            self.trigger.clear()

    def uasub_callback(self, msg):
        # Receives the gaze tracking information as a certainty percentage
        # If at least 80% certain, do the crossing
        self.gaze = msg.data >= 0.8
        if self.gaze and self.ready:
            self.trigger.set()
        else:
            self.trigger.clear()

    def ucsub_callback(self, msg):
        # Allows changing of dest whilst waiting to cross.
        self.new_dest = msg.data

class BeginCrossing(smach.State):
    """ The state where we tell the base to start the move op. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted'],
                             input_keys=['begincross_dest_in'],
                             output_keys=['begincross_dest_out'])

    def execute(self, userdata):
        global client
        if rospy.is_shutdown(): # pragma: no cover
            return 'preempted'
        rospy.loginfo('Executing state BEGINCROSSING dest: ' + str(userdata.begincross_dest_in))

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        goal = CrossRoadGoal()
        goal.crossing_id = userdata.begincross_dest_in
        rospy.loginfo('Asking base to drive to ' + str(goal.crossing_id))
        client.send_goal(goal)

        return 'succeeded'

class Crossing(smach.State):
    """ The state where wheely is moving across the road. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted','replanned','interrupted'],
                             input_keys=['cross_dest_in'],
                             output_keys=['cross_dest_out','cross_midcross_out'])
        self.command = -1
        self.sub = rospy.Subscriber('user_commands', std_msgs.msg.Int8, self.sub_callback)

    def execute(self, userdata):
        global subs,client
        if rospy.is_shutdown(): # pragma: no cover
            return 'preempted'
        rospy.loginfo('Executing state CROSSING')

        # Clear out any commands we should have already acted on.
        self.command = -1

        userdata.cross_midcross_out = 0

        TIMEOUT = 45.0
        TIMESTEP = 0.25
        
        for i in range(int(TIMEOUT/TIMESTEP)):
            if self.preempt_requested():
                client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.command != -1 and self.command < 2:
                client.cancel_goal()
                #userdata.cross_dest_out = self.command
                client.wait_for_result(rospy.Duration.from_sec(10.0))
                res = client.get_result()
                rospy.loginfo('REPLAN RES: ' + str(res))
                userdata.cross_midcross_out = res.location
                userdata.cross_dest_out = self.command
                return 'replanned'
            elif not subs.cs_ready:
                # Lights have changed while we were crossing
                client.cancel_goal()
                client.wait_for_result(rospy.Duration.from_sec(10.0))
                res = client.get_result()
                rospy.loginfo('TOO SLOW RES: ' + str(res))
                userdata.cross_midcross_out = res.location
                userdata.cross_dest_out = userdata.cross_dest_in
                #subs.cs_ready = True
                return 'interrupted'

            finished = client.wait_for_result(rospy.Duration.from_sec(TIMESTEP))
            if finished:
                userdata.cross_midcross_out = -1
                break
        res = client.get_result()
        rospy.loginfo(res)

        return 'succeeded'

    def sub_callback(self, data):
        self.command = data.data

class Retreating(smach.State):
    """ The state where wheely is getting off the road as the lights have changed. """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted','paused'],
                             input_keys=['rtrt_dest_in','rtrt_midcross_in'],
                             output_keys=['rtrt_dest_out','rtrt_midcross_out'])

    def execute(self, userdata):
        global subs,client
        if rospy.is_shutdown(): # pragma: no cover
            return 'preempted'
        rospy.loginfo('Executing state RETREATING')

        print userdata.rtrt_midcross_in, userdata.rtrt_dest_in
        pos = userdata.rtrt_midcross_in
        # Turn around only if less than this far of the way across, to simulate the
        # extra time required to turn around
        turnback_ratio = 0.4
        if rospy.get_param('/wheely_sim/has_island', 0):
            stops = [0.0,0.5,1.0]
        else:
            stops = [0.0,1.0]

        for l,r in pairwise(stops):
            if r > pos:
                break

        gap = r - l
        cutoff = turnback_ratio * gap
        print r,l,cutoff,pos
        if r - pos < cutoff:
            retreat_to = r
        elif pos - l < cutoff:
            retreat_to = l
        else:
            retreat_to = userdata.rtrt_dest_in
        rospy.loginfo('Retreating to ' + str(retreat_to))

        differs = userdata.rtrt_dest_in != retreat_to

        goal = CrossRoadGoal()
        goal.crossing_id = retreat_to
        rospy.loginfo('Asking base to drive to ' + str(goal.crossing_id))
        client.send_goal(goal)

        TIMEOUT = 45.0
        TIMESTEP = 0.25
        
        for i in range(int(TIMEOUT/TIMESTEP)):
            if self.preempt_requested():
                client.cancel_goal()
                self.service_preempt()
                return 'preempted'

            finished = client.wait_for_result(rospy.Duration.from_sec(TIMESTEP))
            if finished:
                break
        res = client.get_result()
        rospy.loginfo(res)

        userdata.rtrt_midcross_out = -1

        if differs:
            userdata.rtrt_dest_out = userdata.rtrt_dest_in
            return 'paused'
        else:
            return 'succeeded'

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

    # Use a global, better than forcing smach to pass this around
    global subs
    subs = SubscriberContainer()
    global client
    client = actionlib.SimpleActionClient('cross_road', CrossRoadAction)
    client.wait_for_server()

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
                                   transitions={'cross':'BEGINCROSSING',
                                                'timeout':'WAITING',
                                                'preempted':'WAITING'},
                                   remapping={'sigwait_dest_in':'user_dest',
                                              'sigwait_midcross_in':'is_midcross'})
            smach.StateMachine.add('BEGINCROSSING', BeginCrossing(),
                                   transitions={'succeeded':'CROSSING',
                                                'preempted':'WAITING'},
                                   remapping={'begincross_dest_in':'user_dest',
                                              'begincross_dest_out':'user_dest'})
            smach.StateMachine.add('CROSSING', Crossing(),
                                   transitions={'succeeded':'WAITING',
                                                'preempted':'WAITING',
                                                'replanned':'SIGNALWAITING',
                                                'interrupted':'RETREATING'},
                                   remapping={'cross_dest_out':'user_dest',
                                              'cross_midcross_out':'is_midcross',
                                              'cross_dest_in':'user_dest'})
            smach.StateMachine.add('RETREATING', Retreating(),
                                   transitions={'succeeded':'WAITING',
                                                'preempted':'WAITING',
                                                'paused':'SIGNALWAITING'},
                                   remapping={'rtrt_midcross_in':'is_midcross',
                                              'rtrt_dest_in':'user_dest',
                                              'rtrt_dest_out':'user_dest',
                                              'rtrt_midcross_out':'is_midcross'})
                                                

        smach.Concurrence.add('MAIN', sm)

    sis = smach_ros.IntrospectionServer('wheely_intro', sm, '/WHEELY_SM')
    sis.start()

    # Execute smach plan!
    outcome = sm_con.execute()

    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except smach.exceptions.InvalidUserCodeError as ex: # pragma: no cover
        if rospy.is_shutdown():
            print 'Shutting down'
        else:
            raise
