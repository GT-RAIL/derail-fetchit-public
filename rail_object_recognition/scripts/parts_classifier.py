#!/usr/bin/env python

import os
import pandas

import numpy as np
import pickle

import rospy
import rospkg

from rail_object_recognition.srv import PartsQuery, PartsQueryResponse


class PartsClassifier:
    def __init__(self):
        self.scalar = None
        self.classifier = None
        # Get the data_filepath and test_filepath. We want things to break if
        # the appropriate params are not set
        model_filepath = rospy.get_param('~model_filepath')
        self.load_classifier(model_filepath)

        self.server = rospy.Service("rail_object_recognition/classify_parts", PartsQuery, self.classify)

    def classify(self, req):
        X_test = np.array([item.descriptor for item in req.descriptors])
        if len(X_test.shape) == 1:
            X_test = X_test.reshape(1, -1)

        rospy.loginfo("object detector input:\n{}".format(X_test))

        rospy.loginfo("Data Shape: {}".format(X_test.shape))
        X_test = self.scalar.transform(X_test)

        rospy.loginfo("object detector input:\n{}".format(X_test))

        classifications = self.classifier.predict_proba(X_test)
        rospy.loginfo("Classifications:\n{}".format(classifications))

        return PartsQueryResponse(classifications.reshape(-1).tolist())

    def load_classifier(self, model_filepath):
    	dic = pickle.load(open(model_filepath, "rb"))
    	self.scalar = dic["feature_normalization_scalar"]
    	self.classifier = dic["classifier"]

if __name__ == "__main__":
    rospy.init_node("parts_classifier_server")
    parts_classifier = PartsClassifier()
    rospy.spin()
