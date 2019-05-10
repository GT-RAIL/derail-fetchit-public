#! /usr/bin/env python

import rospy
import actionlib
import data_recorder.msg

class ExecutorClient:
    def __init__(self):
        client = actionlib.SimpleActionClient('stir', data_recorder.msg.PlaybackAction)
        print "HI"

        client.wait_for_server()
        print "HI2"

        goal = data_recorder.msg.PlaybackGoal()
        client.send_goal(goal)

        client.wait_for_result()

        print client.get_state()


if __name__ == '__main__':
    rospy.init_node('executor_client_node')
    executor_client = ExecutorClient()
