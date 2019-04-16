import math

import rospy
from tf.transformations import quaternion_about_axis
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from yaml import load, YAMLError

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
            try:
                wp_yaml = load(stream)
                return wp_yaml
            except YAMLError as exc:
                rospy.logerr(rospy.get_name() + ": " + str(exc))
                return False

    def parse_map_yaml(self, yaml):
        map_wps = {}

        # gets map name and frame
        if not 'map_name' in yaml:
            return False
        self.map_name = yaml['map_name']
        if not 'map_frame' in yaml:
            return False
        self.map_frame = yaml['map_frame']

        # checks if waypoints missing
        if not 'map_waypoints' in yaml:
            rospy.logwarn(rospy.get_name() + ": 'map_waypoints' not specified in waypoint map yaml")
            return map_wps
        if len(yaml['map_waypoints']) == 0:
            rospy.logwarn(rospy.get_name() + ": 'map_waypoints' are empty")
            return map_wps

        # gets waypoints
        for waypoint in yaml['map_waypoints']:
            # checks for frame label
            if not 'label' in waypoint:
                rospy.logwarn(rospy.get_name() + ": waypoint missing label, skipping")
                continue
            if waypoint['label'] in self.map_wps:
                rospy.logwarn(rospy.get_name() + ": waypoint repeated, overwriting")
                continue

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
            static_transformStamped.transform.translation.z = waypoint['position'][2]

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
        for wp_labl, wp_tfStamped in self.map_wps.items():
            broadcaster.sendTransform(wp_tfStamped)