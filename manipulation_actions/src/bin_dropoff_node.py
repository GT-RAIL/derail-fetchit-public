#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image, PointCloud2

def image_callback(raw_img):
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

    


def bin_dropoff_node():
    """
    Takes in an image coordinate and determines the world coordinate.
    """

    rospy.init_node('listener')

    rospy.Subscriber('head_camera/rgb/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    bin_dropoff_node()