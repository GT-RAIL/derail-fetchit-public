#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2

def center_callback(index):
    pointcloud_data = rospy.wait_for_message("head_camera/depth_registered/points", PointCloud2)
    coordinates_arr = ()
    for i in range(-3, 3):
        if not (pointcloud_data.data[index].x + i == 0 and pointcloud_data.data[index].y + i == 0 and pointcloud_data.data[index].z + i == 0):
            coordinates_arr.append((pointcloud_data.data[index].x + i, pointcloud_data.data[index].y + i, pointcloud_data.data[index].z + i))

    averaged_point = [sum(y) / len(y) for y in zip(*coordinates_arr)]


def bin_dropoff_node():
    """
    Takes in an image coordinate and determines the world coordinate.
    """

    rospy.init_node('listener')

    rospy.Subscriber('square_center_coord', Int32, center_callback)
    rospy.spin()

if __name__ == "__main__":
    bin_dropoff_node()