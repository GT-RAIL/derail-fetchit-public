#!/usr/bin/env python
# The node for the action server that executes background behaviours. This is
# not connected to the arbitration node, and it cannot run its own background
# task

import rospy

from task_executor.server import TaskServer
from task_executor.actions import default_actions_dict, Actions


BACKGROUND_TASK_ACTION = 'background_task'


def main():
    global BACKGROUND_TASK_ACTION

    rospy.init_node('idle_executor')

    # Instantiate the actions
    del default_actions_dict[BACKGROUND_TASK_ACTION]
    actions = Actions(default_actions_dict)
    server = TaskServer(actions=actions, connect_arbitrator=False)
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
