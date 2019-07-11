#!/usr/bin/env python
# The node for running affordance_net

import rospy

from rail_semantic_grasping.data_collection import DataCollection


def main():
    rospy.init_node('data_collection')
    dc = DataCollection()
    rospy.spin()


if __name__ == '__main__':
    main()
