#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_monitor.server import AssistanceArbitrationServer

def main():
    rospy.init_node('arbitrator')
    server = AssistanceArbitrationServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
