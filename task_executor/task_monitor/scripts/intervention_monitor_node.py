#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from task_monitor.intervention_monitor import InterventionMonitor

def main():
    rospy.init_node('intervention_monitor')
    server = InterventionMonitor()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
