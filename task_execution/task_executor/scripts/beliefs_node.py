#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_executor.beliefs import BeliefsServer


def main():
    rospy.init_node('beliefs')
    server = BeliefsServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
