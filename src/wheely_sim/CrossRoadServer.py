#!/usr/local/bin/coverage run

import rospy
import actionlib

from wheely_sim.msg import CrossRoadAction, CrossRoadResult, CrossRoadFeedback
from geometry_msgs.msg import Twist # For simple motor control

class CrossRoadServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('cross_road', CrossRoadAction, self.execute, False)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.cmd_vel_fwd = Twist()
        self.cmd_vel_fwd.linear.x = 1.0

        self.cmd_vel_rev = Twist()
        self.cmd_vel_rev.linear.x = -1.0

        self.SLEEP_RATE = 5.0 # Hz
        self.MOVE_STEPS = 30 # 6 seconds * 1m/s = 6m
        self.location = 0 # Use 0.5 for island?

        self.server.start()

    def goalid_to_location(self,id):
        goal2loc = {
            0.0:0,
            1.0:self.MOVE_STEPS,
            0.5:self.MOVE_STEPS / 2
        }
        return goal2loc[id]
        # if id:
        #     return self.MOVE_STEPS - 1
        # else:
        #     return 0

    def execute(self,goal):
        gloc = self.goalid_to_location(goal.crossing_id)
        if gloc == self.location:
            rospy.loginfo('Cross road not necessary.')
            res = CrossRoadResult()
            res.did_we_make_it = True
            res.pcnt_prog = 1.0
            res.location = self.location / float(self.MOVE_STEPS - 1)
            self.server.set_succeeded(result = res)
            return
        step_dist = gloc - self.location
        if step_dist > 0:
            drive_cmd = self.cmd_vel_fwd
        else:
            drive_cmd = self.cmd_vel_rev
        rate = rospy.Rate(self.SLEEP_RATE)
        for i in range(abs(step_dist)):
            if self.server.is_preempt_requested():
                # Cancel movement, brake. Warning: this is vulnerable to drift!
                rospy.loginfo('Cross road cancelled.')
                self.pub.publish(Twist())
                res = CrossRoadResult()
                res.did_we_make_it = 0
                res.pcnt_prog = float(self.location) / abs(step_dist)
                res.location = self.location / float(self.MOVE_STEPS - 1)
                self.server.set_preempted(result = res)
                return
            self.pub.publish(drive_cmd)
            self.location = self.location + cmp(step_dist,0) # Use cmp to get sign
            fdbk = CrossRoadFeedback()
            fdbk.pcnt_prog = float(self.location) / abs(step_dist)
            fdbk.location = self.location / float(self.MOVE_STEPS - 1)
            self.server.publish_feedback(feedback = fdbk)
            rate.sleep()
        self.pub.publish(Twist()) # brake
        rospy.loginfo('Braking.')
        res = CrossRoadResult()
        res.did_we_make_it = 1
        res.pcnt_prog = 1.0
        res.location = self.location / float(self.MOVE_STEPS - 1)
        self.server.set_succeeded(result = res)

if __name__ == '__main__':
    rospy.init_node('cross_road_server')
    server = CrossRoadServer()
    rospy.spin()
