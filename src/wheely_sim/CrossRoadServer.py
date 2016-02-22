#!/usr/bin/env python

import roslib
roslib.load_manifest('wheely_sim')
import rospy
import actionlib

from wheely_sim.msg import CrossRoadAction

class CrossRoadServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('cross_road', CrossRoadAction, self.execute, False)
        self.server.start()

    def execute(self,goal):
        # TODO: This should do the action
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('cross_road_server')
    server = CrossRoadServer()
    rospy.spin()
