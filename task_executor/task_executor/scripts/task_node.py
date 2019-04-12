#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_executor.server import TaskServer


def main():
    rospy.init_node('task_executor')
    server = TaskServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
