#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_monitor.server import TaskMonitorServer

def main():
    rospy.init_node('task_monitor')
    server = TaskMonitorServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
