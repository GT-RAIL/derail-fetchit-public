#!/usr/bin/env python
# Relay the octomap

import rospy

from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap


if __name__ == '__main__':
    rospy.init_node('relay_node')
    p = rospy.Publisher('/octomap', Octomap, queue_size=10)

    def relay(msg):
        octo = msg.world.octomap.octomap
        octo.header.frame_id = msg.world.octomap.header.frame_id
        # octo.header.stamp = rospy.Time.now()
        p.publish(octo)

    s = rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, relay)
    rospy.spin()
