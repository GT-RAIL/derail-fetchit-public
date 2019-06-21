#!/usr/bin/env python
# The node for running affordance_net

import rospy

from rail_part_affordance_detection.affordance_net import AffordanceNet


def main():
    rospy.init_node('affordance_detection')
    afnet = AffordanceNet()
    rospy.spin()


if __name__ == '__main__':
    main()
