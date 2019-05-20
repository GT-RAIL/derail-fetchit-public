#!/usr/bin/env python
import rospy
import actionlib
import cv2
import imutils
from sensor_msgs.msg import Image, PointCloud2
from manipulation_actions import BinDropoffAction, BinDropoffResult, LinearMoveAction, LinearMoveGoal
from geometry_msgs import Point, Vector3

class BinDropoff(object):
    _result = BinDropoffResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("bin_dropoff", BinDropoffAction, execute_cb = self.image_callback, auto_start = False)
        self.linear_client = actionlib.SimpleActionClient("linear_controller/linear_move", LinearMoveAction)


    def image_callback(raw_img):
        raw_img = rospy.wait_for_message("head_camera/rgb/image_raw", Image)
        gray_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.GaussianBlur(gray_img, (9, 9), 0)
        thresh = cv2.threshold(blurred_img, 60, 255, cv2.THRESH_BINARY_INV)[1]

        # find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # loop over the contours
        largest_area = 0
        _cX = 0
        _cY = 0
        _c = None
        _x = None
        _y = None
        _w = None
        _h = None
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) != 4:
                continue
            (x, y, w, h) = cv2.boundingRect(approx)
            print("area is:", w * h)
            if w * h < largest_area:
                continue
            largest_area = w * h

            # compute the center of the contour
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            _cX = cX
            _cY = cY
            _c = c
            _x = x
            _y = y
            _w = w
            _h = h

        index = _cY * raw_img.shape[1] + _cX

        pointcloud_data = rospy.wait_for_message("head_camera/depth_registered/points", PointCloud2)
        coordinates_arr = ()
        for i in range(-3, 3):
            if not (pointcloud_data.data[index].x + i == 0 and pointcloud_data.data[index].y + i == 0 and pointcloud_data.data[index].z + i == 0):
                coordinates_arr.append((pointcloud_data.data[index].x + i, pointcloud_data.data[index].y + i, pointcloud_data.data[index].z + i))

        averaged_point = [sum(y) / len(y) for y in zip(*coordinates_arr)]

        linear_goal = LinearMoveGoal()
        linear_goal.point.x = averaged_point[0]
        linear_goal.point.y = averaged_point[1]
        linear_goal.point.z = averaged_point[2]
        linear_goal.hold_final_pose = True
        self.linear_client.send_goal(linear_goal)
        self.linear_client.wait_for_result()
    

if __name__ == "__main__":
    rospy.init_node("bin_dropoff")
    bin_dropoff = BinDropoff()
    rospy.spin()
