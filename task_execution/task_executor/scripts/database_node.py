#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_executor.database import DatabaseServer


def main():
    rospy.init_node('database')
    server = DatabaseServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
