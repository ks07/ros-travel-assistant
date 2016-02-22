#!/usr/bin/env python

import roslib
roslib.load_manifest('wheely_sim')
import rospy
import actionlib

from wheely_sim.msg import CrossRoadAction, CrossRoadResult

class CrossRoadServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('cross_road', CrossRoadAction, self.execute, False)
        self.server.start()

    def execute(self,goal):
        # TODO: This should do the action
        res = CrossRoadResult()
        res.did_we_make_it = 197
        self.server.set_succeeded(result = res)

if __name__ == '__main__':
    rospy.init_node('cross_road_server')
    server = CrossRoadServer()
    rospy.spin()
