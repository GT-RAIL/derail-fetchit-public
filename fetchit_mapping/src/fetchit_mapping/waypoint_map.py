import math

import rospy
from tf.transformations import quaternion_about_axis
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from ruamel.yaml import YAML


# YAML init
yaml = YAML(typ='safe')


class WayPointMap:
    def __init__(self, map_fp):
        # loads the waypoints from yaml
        map_yaml = self.load_map_yaml(map_fp)
        if not map_yaml:
            rospy.logfatal(rospy.get_name() + ": cannot load map YAML. Waypoint system failure.")
            exit()

        # builds waypoint map data structure
        self.map_wps = self.parse_map_yaml(map_yaml)
        if not self.map_wps:
            rospy.logfatal(rospy.get_name() + ": cannot parse map YAML. Waypoint system failure.")
            exit()

        # publishes all waypoints as tfs
        self.start_static_broadcasters()


    def load_map_yaml(self, filepath):
        with open(filepath, 'r') as stream:
            wp_yaml = yaml.load(stream)
            return wp_yaml

    def parse_map_yaml(self, wp_yaml):
        map_wps = {}

        # gets map name and frame. throw an exception if these keys do not exist
        self.map_name = wp_yaml['map_name']
        self.map_frame = wp_yaml['map_frame']

        # checks if waypoints missing
        if not 'map_waypoints' in wp_yaml:
            rospy.logwarn(rospy.get_name() + ": 'map_waypoints' not specified in waypoint map wp_yaml")
            return map_wps
        if len(wp_yaml['map_waypoints']) == 0:
            rospy.logwarn(rospy.get_name() + ": 'map_waypoints' are empty")
            return map_wps

        # gets waypoints
        for waypoint in wp_yaml['map_waypoints']:
            # checks for frame label
            if not 'label' in waypoint:
                rospy.logwarn(rospy.get_name() + ": waypoint missing label, skipping")
                continue
            if waypoint['label'] in map_wps:
                rospy.logwarn(rospy.get_name() + ": waypoint {} repeated, overwriting".format(waypoint['label']))

            # creates static transform
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = self.map_frame
            static_transformStamped.child_frame_id = waypoint['label']

            # extracts position
            if not 'position' in waypoint:
                rospy.logwarn(rospy.get_name() + ": waypoint missing position, skipping")
                continue
            static_transformStamped.transform.translation.x = waypoint['position'][0]
            static_transformStamped.transform.translation.y = waypoint['position'][1]
            static_transformStamped.transform.translation.z = 0.0

            # extracts orientation
            if not 'orientation' in waypoint:
                rospy.logwarn(rospy.get_name() + ": waypoint missing orientation, skipping")
                continue
            yaw_angle = math.radians(waypoint['orientation'])
            quat = quaternion_about_axis(yaw_angle,(0,0,1))
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]

            # adds to waypoints
            map_wps[waypoint['label']] = static_transformStamped

        return map_wps


    def start_static_broadcasters(self):
        broadcaster = StaticTransformBroadcaster()
        broadcaster.sendTransform(self.map_wps.values())
