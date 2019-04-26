#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_monitor.execution_monitor import ExecutionMonitor

def main():
    rospy.init_node('execution_monitor')
    server = ExecutionMonitor()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
