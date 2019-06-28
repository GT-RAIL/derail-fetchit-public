#!/usr/bin/env python

import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect2
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer

import numpy as np
import os
import caffe
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Pose, Point
# from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from rail_part_affordance_detection.msg import ObjectPartAffordance
from rail_part_affordance_detection.srv import DetectAffordances, DetectAffordancesResponse


class AffordanceNet:

    DETECT_AFFORDANCES_SERVICE_NAME = "rail_part_affordance_detection/detect"

    # Visualization parameters for object detection
    OBJECT_CLASSES = ('__background__', 'bowl', 'tvm', 'pan', 'hammer', 'knife', 'cup', 'drill', 'racket', 'spatula',
                   'bottle')
    col0 = [0, 0, 0]
    col1 = [0, 255, 255]
    col2 = [255, 0, 255]
    col3 = [0, 125, 255]
    col4 = [55, 125, 0]
    col5 = [255, 50, 75]
    col6 = [100, 100, 50]
    col7 = [25, 234, 54]
    col8 = [156, 65, 15]
    col9 = [215, 25, 155]
    col10 = [25, 25, 155]
    OBJECT_COLORS = [col0, col1, col2, col3, col4, col5, col6, col7, col8, col9, col10]

    # # Visualization parameters for part affordance detection
    background = [200, 222, 250]
    c1 = [0, 0, 205]
    c2 = [34, 139, 34]
    c3 = [192, 192, 128]
    c4 = [165, 42, 42]
    c5 = [128, 64, 128]
    c6 = [204, 102, 0]
    c7 = [184, 134, 11]
    c8 = [0, 153, 153]
    c9 = [0, 134, 141]
    c10 = [184, 0, 141]
    c11 = [184, 134, 0]
    c12 = [184, 134, 223]
    AFFORDANCE_COLORS = np.array([background, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12])
    AFFORDANCE_LABELS = ['background', 'contain', 'cut', 'display', 'engine', 'grasp', 'hit', 'pound', 'support',
                         'w-grasp']

    def __init__(self):

        # Detection params
        self.confidence_threshold = rospy.get_param('~confidence_threshold')
        self.good_range = rospy.get_param('~good_range')

        # Model
        self.model_filepath = rospy.get_param('~model_filepath')
        self.prototxt_filepath = rospy.get_param('~prototxt_filepath')

        # Visualize
        self.visualize = rospy.get_param('~visualize')

        # Initialize image subscribers
        self.header = None
        self.height = None
        self.width = None
        self.bgr_img = None
        self.dep_img = None
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber(rospy.get_param('~camera_color_topic'), Image, self.color_image_callback)
        self.dep_sub = rospy.Subscriber(rospy.get_param('~camera_depth_topic'), Image, self.depth_image_callback)

        # Initialize server for detecting affordances
        self.server = rospy.Service(AffordanceNet.DETECT_AFFORDANCES_SERVICE_NAME, DetectAffordances,
                                    self.detect)
        rospy.loginfo("Started {} server".format(AffordanceNet.DETECT_AFFORDANCES_SERVICE_NAME))

    def color_image_callback(self, data):
        try:
            # bgr8 is the image encoding expected by most opencv function
            self.header, self.height, self.width = data.header, data.height, data.width
            self.bgr_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_image_callback(self, dep_msg):
        try:
            self.dep_img = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect(self, req):
        ########################################################################
        # IMPORTANT: the pycaffe network needs to be loaded inside of the service handler. Otherwise, pycaffe calls will
        #            be extremely slow. One possible reason is that threads that handle an incoming service request
        #            cannot access the state instance of the main thread, so they have to (re)create / copy one of
        #            their own.
        cfg.TEST.HAS_RPN = True  # Use RPN for proposals
        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
        if not os.path.isfile(self.model_filepath):
            raise IOError("{:s} not found.".format(self.model_filepath))
        net = caffe.Net(self.prototxt_filepath, self.model_filepath, caffe.TEST)
        ########################################################################

        if self.bgr_img is not None and self.dep_img is not None:
            h, w, c = self.bgr_img.shape
            if h > 100 and w > 100:
                # Detect all object classes and regress object bounds
                timer = Timer()
                timer.tic()
                if cfg.TEST.MASK_REG:
                    rois_final, rois_class_score, rois_class_ind, masks, scores, boxes = im_detect2(net, self.bgr_img)
                    timer.toc()
                    rospy.loginfo('Detection took {:.3f}s for {:d} object proposals'.format(timer.total_time,
                                                                                            rois_final.shape[0]))
                    list_boxes, list_masks = self.visualize_mask(self.bgr_img, rois_final, rois_class_score,
                                                                 rois_class_ind, masks)
                    if list_boxes and list_masks:
                        object_part_affordance_list = []
                        for i in range(len(list_boxes)):
                            box = list_boxes[i]
                            rospy.loginfo(box)
                            mask = list_masks[i]
                            object_part_affordance = ObjectPartAffordance()
                            object_part_affordance.header.stamp = rospy.Time.now()
                            object_part_affordance.header.frame_id = self.header.frame_id
                            object_part_affordance.object_class = box[0]
                            object_part_affordance.detection_score = box[1]
                            object_part_affordance.bounding_box = box[2:]
                            object_part_affordance.height = self.height
                            object_part_affordance.width = self.width
                            object_part_affordance.affordance_mask = mask.flatten().tolist()
                            object_part_affordance_list.append(object_part_affordance)
                        return DetectAffordancesResponse(success=True,
                                                         object_part_affordance_list=object_part_affordance_list)

        return DetectAffordancesResponse(success=False)

    def visualize_mask(self, im, rois_final, rois_class_score, rois_class_ind, masks):
        list_bboxes = []
        list_masks = []
        if rois_final.shape[0] == 0:
            return list_bboxes, list_masks
        inds = np.where(rois_class_score[:, -1] >= self.confidence_threshold)[0]
        if len(inds) == 0:
            rospy.loginfo('No detected box with probality > thresh = {} -- Choossing highest confidence bounding box.'.
                          format(self.confidence_threshold))
            inds = [np.argmax(rois_class_score)]
            max_conf = np.max(rois_class_score)
            if max_conf < 0.001:
                return list_bboxes, list_masks
        rois_final = rois_final[inds, :]
        rois_class_score = rois_class_score[inds, :]
        rois_class_ind = rois_class_ind[inds, :]

        # get mask
        masks = masks[inds, :, :, :]
        im_width = im.shape[1]
        im_height = im.shape[0]
        # transpose BGR channel back to RGB
        im = im[:, :, (2, 1, 0)]
        num_boxes = rois_final.shape[0]

        for i in range(0, num_boxes):
            curr_mask = np.full((im_height, im_width), 0.0, 'float')  # convert to int later
            class_id = int(rois_class_ind[i, 0])
            bbox = rois_final[i, 1:5]
            score = rois_class_score[i, 0]

            if cfg.TEST.MASK_REG:
                x1 = int(round(bbox[0]))
                y1 = int(round(bbox[1]))
                x2 = int(round(bbox[2]))
                y2 = int(round(bbox[3]))
                x1 = np.min((im_width - 1, np.max((0, x1))))
                y1 = np.min((im_height - 1, np.max((0, y1))))
                x2 = np.min((im_width - 1, np.max((0, x2))))
                y2 = np.min((im_height - 1, np.max((0, y2))))

                # bboxes format: class_id, probability, x1, y1, x2, y2
                cur_box = [class_id, score, x1, y1, x2, y2]
                list_bboxes.append(cur_box)

                h = y2 - y1
                w = x2 - x1
                mask = masks[i, :, :, :]
                mask = np.argmax(mask, axis=0)
                original_uni_ids = np.unique(mask)
                # sort before_uni_ids and reset [0, 1, 7] to [0, 1, 2]
                original_uni_ids.sort()
                mask = self.reset_mask_ids(mask, original_uni_ids)
                mask = cv2.resize(mask.astype('float'), (int(w), int(h)), interpolation=cv2.INTER_LINEAR)
                # mask = convert_mask_to_original_ids(mask, original_uni_ids)
                mask = self.convert_mask_to_original_ids_manual(mask, original_uni_ids)
                # for mult masks
                curr_mask[y1:y2, x1:x2] = mask

                # each curr_mask has the same size as the input image
                curr_mask = curr_mask.astype('uint8')
                list_masks.append(curr_mask)

                if self.visualize:
                    color_curr_mask = AffordanceNet.AFFORDANCE_COLORS.take(curr_mask, axis=0).astype('uint8')
                    cv2.imshow('Part affordance ' + str(i), color_curr_mask)
                    # cv2.imwrite('mask'+str(i)+'.jpg', color_curr_mask)

        if self.visualize:
            img_org = im.copy()
            for ab in list_bboxes:
                img_out = self.draw_reg_text(img_org, ab)
            cv2.imshow('Object detection', img_out)
            legend = self.show_color_legend()
            cv2.imshow('Legend', legend)
            rospy.loginfo("Press any key to close all windows and exit the service call.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return list_bboxes, list_masks

    def reset_mask_ids(self, mask, before_uni_ids):
        # reset ID mask values from [0, 1, 4] to [0, 1, 2] to resize later
        counter = 0
        for id in before_uni_ids:
            mask[mask == id] = counter
            counter += 1
        return mask

    def convert_mask_to_original_ids_manual(self, mask, original_uni_ids):
        # TODO: speed up!!!
        temp_mask = np.copy(mask)  # create temp mask to do np.around()
        temp_mask = np.around(temp_mask, decimals=0)  # round 1.6 -> 2., 1.1 -> 1.
        current_uni_ids = np.unique(temp_mask)
        out_mask = np.full(mask.shape, 0, 'float32')
        mh, mw = mask.shape
        for i in range(mh - 1):
            for j in range(mw - 1):
                for k in range(1, len(current_uni_ids)):
                    if (current_uni_ids[k] - self.good_range) < mask[i][j] < (
                            current_uni_ids[k] + self.good_range):
                        out_mask[i][j] = original_uni_ids[k]
                        # mask[i][j] = current_uni_ids[k]
        #     const = 0.005
        #     out_mask = original_uni_ids[(np.abs(mask - original_uni_ids[:,None,None]) < const).argmax(0)]
        return out_mask

    def show_color_legend(self):
        # initialize the legend visualization
        legend = np.zeros(((len(AffordanceNet.AFFORDANCE_LABELS) * 25) + 25, 200, 3), dtype="uint8")

        # loop over the class names + colors
        for i, label in enumerate(AffordanceNet.AFFORDANCE_LABELS):
            # draw the class name + color on the legend
            color = AffordanceNet.AFFORDANCE_COLORS[i]
            cv2.putText(legend, label, (5, (i * 25) + 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.rectangle(legend, (100, (i * 25)), (200, (i * 25) + 25),
                          tuple(color), -1)
        return legend

    def draw_reg_text(self, img, obj_info):
        obj_id = obj_info[0]
        cfd = obj_info[1]
        xmin = obj_info[2]
        ymin = obj_info[3]
        xmax = obj_info[4]
        ymax = obj_info[5]
        self.draw_arrow(img, (xmin, ymin), (xmax, ymin), AffordanceNet.OBJECT_COLORS[obj_id], 0, 5, 8, 0)
        self.draw_arrow(img, (xmax, ymin), (xmax, ymax), AffordanceNet.OBJECT_COLORS[obj_id], 0, 5, 8, 0)
        self.draw_arrow(img, (xmax, ymax), (xmin, ymax), AffordanceNet.OBJECT_COLORS[obj_id], 0, 5, 8, 0)
        self.draw_arrow(img, (xmin, ymax), (xmin, ymin), AffordanceNet.OBJECT_COLORS[obj_id], 0, 5, 8, 0)
        # put text
        txt_obj = AffordanceNet.OBJECT_CLASSES[obj_id] + ' ' + str(cfd)
        cv2.putText(img, txt_obj, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)  # draw with red
        return img

    def draw_arrow(self, image, p, q, color, arrow_magnitude, thickness, line_type, shift):
        # draw arrow tail
        cv2.line(image, p, q, color, thickness, line_type, shift)
        # calc angle of the arrow
        angle = np.arctan2(p[1] - q[1], p[0] - q[0])
        # starting point of first line of arrow head
        p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi / 4)),
             int(q[1] + arrow_magnitude * np.sin(angle + np.pi / 4)))
        # draw first half of arrow head
        cv2.line(image, p, q, color, thickness, line_type, shift)
        # starting point of second line of arrow head
        p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi / 4)),
             int(q[1] + arrow_magnitude * np.sin(angle - np.pi / 4)))
        # draw second half of arrow head
        cv2.line(image, p, q, color, thickness, line_type, shift)
