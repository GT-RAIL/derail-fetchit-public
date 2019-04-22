#!/usr/bin/env python
# The node for the action server that executes background behaviours. This is
# not connected to the arbitration node, and it cannot run its own background
# task

import rospy

from task_executor.server import TaskServer
from task_executor.actions import default_actions_dict, Actions


def main():
    rospy.init_node('recovery_executor')
    server = TaskServer(connect_monitor=False)
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
