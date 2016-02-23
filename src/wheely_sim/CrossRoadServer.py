#!/usr/bin/env python

import roslib
roslib.load_manifest('wheely_sim')
import rospy
import actionlib

from wheely_sim.msg import CrossRoadAction, CrossRoadResult
from geometry_msgs.msg import Twist # For simple motor control

class CrossRoadServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('cross_road', CrossRoadAction, self.execute, False)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.cmd_vel_fwd = Twist()
        self.cmd_vel_fwd.linear.x = 1.0

        self.server.start()

    def execute(self,goal):
        # TODO: This should do the action
        rate = rospy.Rate(5.0) # 5 hz
        for i in range(100): # 20 secs
            self.pub.publish(self.cmd_vel_fwd)
            rate.sleep()
        self.pub.publish(Twist()) # brake
        res = CrossRoadResult()
        res.did_we_make_it = 197
        self.server.set_succeeded(result = res)

if __name__ == '__main__':
    rospy.init_node('cross_road_server')
    server = CrossRoadServer()
    rospy.spin()
