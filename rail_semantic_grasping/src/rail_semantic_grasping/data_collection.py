#!/usr/bin/env python

import os
import shutil
import glob
from datetime import datetime
import pickle

import rospy
import rospkg
from rail_semantic_grasping.msg import SemanticObjectList
# from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, PoseStamped
# from sensor_msgs.msg import Image
# from rail_part_affordance_detection.msg import ObjectPartAffordance
# from rail_part_affordance_detection.srv import DetectAffordances, DetectAffordancesResponse
from visualization_msgs.msg import MarkerArray


class DataCollection:

    def __init__(self, collect_objects=True):

        # Initialize semantic objects subscribers
        if collect_objects:
            self.semantic_objects_topic = rospy.get_param("~semantic_objects_with_grasps_topic")
        self.data_dir = rospy.get_param("~data_dir_path",
                                        os.path.join(rospkg.RosPack().get_path("rail_semantic_grasping"), "data"))

        # Set up data folder
        if not os.path.exists(self.data_dir):
            os.mkdir(self.data_dir)
            rospy.loginfo("Data folder is set up at {}".format(self.data_dir))
        else:
            rospy.loginfo("Data folder is at {}".format(self.data_dir))

        self.unlabeled_data_dir = os.path.join(self.data_dir, "unlabeled")
        if not os.path.exists(self.unlabeled_data_dir):
            os.mkdir(self.unlabeled_data_dir)

        self.labeled_data_dir = os.path.join(self.data_dir, "labeled")
        if not os.path.exists(self.labeled_data_dir):
            os.mkdir(self.labeled_data_dir)

        # set up session folder
        if collect_objects:
            time = datetime.now()  # fetch time
            date = time.strftime("%Y_%m_%d_%H_%M")
            self.session_dir = os.path.join(self.unlabeled_data_dir, date)
            os.mkdir(self.session_dir)
            rospy.loginfo("Start data collection session---data will be saved to {}".format(self.session_dir))

            self.object_counter = 0

        if collect_objects:
            # Listen to semantic objects with grasps
            self.semantic_objects_sub = rospy.Subscriber(self.semantic_objects_topic,
                                                         SemanticObjectList, self.semantic_objects_callback)
            rospy.loginfo("Listen to semantic objects with grasp from {}".format(self.semantic_objects_topic))
        else:
            # Set up publishers
            self.markers_pub = rospy.Publisher("~data_collection/markers", MarkerArray, queue_size=10, latch=True)
            self.grasp_pub = rospy.Publisher("~data_collection/grasp", PoseStamped, queue_size=10, latch=True)

    def semantic_objects_callback(self, semantic_objects):
        object_file_path = os.path.join(self.session_dir, str(self.object_counter) + ".pkl")
        with open(object_file_path, "wb") as fh:
            pickle.dump(semantic_objects, fh)
            rospy.loginfo("Saved object No.{}".format(self.object_counter))
        self.object_counter += 1

    def collect_grasps(self):
        # grab all sessions in the unlabeled data dir
        session_dirs = glob.glob(os.path.join(self.unlabeled_data_dir, "*"))

        for session_dir in session_dirs:
            object_files = glob.glob(os.path.join(session_dir, "*.pkl"))

            # prepare dir for saving labeled data
            labeled_session_dir = session_dir.replace("unlabeled", "labeled")
            if not os.path.exists(labeled_session_dir):
                os.mkdir(labeled_session_dir)

            # iterate through objects
            for object_file in object_files:
                with open(object_file, "rb") as fh:
                    semantic_objects = pickle.load(fh)
                key = raw_input("Proceed with semantic objects: {}? y/n ".format(object_file))
                if key != "y":
                    continue

                # visualize semantic object
                markers = MarkerArray()
                # assume there is only one object in the list
                rospy.loginfo("{}".format(len(semantic_objects.objects)))
                if not semantic_objects.objects:
                    continue
                semantic_object = semantic_objects.objects[0]
                for semantic_part in semantic_object.parts:
                    markers.markers.append(semantic_part.marker)
                    markers.markers.append(semantic_part.text_marker)
                self.markers_pub.publish(markers)

                # iterate through grasps
                if not semantic_object.grasps:
                    continue
                rospy.loginfo("Current object has {} grasps".format(len(semantic_object.grasps)))
                skip_object = False
                for gi, semantic_grasp in enumerate(semantic_object.grasps):
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = semantic_objects.header.frame_id
                    pose_stamped.pose = semantic_grasp.grasp_pose
                    self.grasp_pub.publish(pose_stamped)
                    print(semantic_grasp.score)
                    rospy.loginfo("Grasp No.{} is on the part with affordance: {}".format(gi, semantic_grasp.grasp_part_affordance))
                    key = raw_input("Is this grasp semantically correct? y/n ")
                    if key == "y":
                        semantic_grasp.score = 1
                    elif key == "n":
                        semantic_grasp.score = 1
                    elif key == "q":
                        skip_object = True
                        break

                if skip_object:
                    continue
                    
                rospy.loginfo("Saving labeled grasps...\n")
                new_object_file = object_file.replace("unlabeled", "labeled")
                with open(new_object_file, "wb") as fh:
                    pickle.dump(semantic_objects, fh)

        rospy.loginfo("All objects has finished labeling. Exiting!")
        exit()

    def visualize_grasps(self):
        # grab all sessions in the labeled data dir
        session_dirs = glob.glob(os.path.join(self.labeled_data_dir, "*"))

        for session_dir in session_dirs:
            object_files = glob.glob(os.path.join(session_dir, "*.pkl"))

            # iterate through objects
            for object_file in object_files:
                with open(object_file, "rb") as fh:
                    semantic_objects = pickle.load(fh)
                key = raw_input("Proceed with semantic objects: {}? y/n ".format(object_file))
                if key != "y":
                    continue

                # visualize semantic object
                markers = MarkerArray()
                # assume there is only one object in the list
                rospy.loginfo("{}".format(len(semantic_objects.objects)))
                if not semantic_objects.objects:
                    continue
                semantic_object = semantic_objects.objects[0]
                for semantic_part in semantic_object.parts:
                    markers.markers.append(semantic_part.marker)
                    markers.markers.append(semantic_part.text_marker)
                self.markers_pub.publish(markers)

                # iterate through grasps
                if not semantic_object.grasps:
                    continue
                rospy.loginfo("Current object has {} grasps".format(len(semantic_object.grasps)))
                for gi, semantic_grasp in enumerate(semantic_object.grasps):
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = semantic_objects.header.frame_id
                    pose_stamped.pose = semantic_grasp.grasp_pose
                    self.grasp_pub.publish(pose_stamped)
                    print(semantic_grasp.score)
                    rospy.loginfo("Grasp No.{} on the part with affordance {} is semantically correct? {}".format(gi, semantic_grasp.grasp_part_affordance, semantic_grasp.score))
                    key = raw_input("enter to continue")

        rospy.loginfo("All objects has finished visualizing. Exiting!")
        exit()
