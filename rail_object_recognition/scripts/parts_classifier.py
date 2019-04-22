#!/usr/bin/env python

import os
import pandas

import numpy as np

from sklearn import preprocessing
from sklearn.metrics import confusion_matrix
from sklearn.neighbors import KNeighborsClassifier
from sklearn.utils import shuffle
from sklearn.externals import joblib

import rospy
import rospkg

from rail_object_recognition.srv import PartsQuery, PartsQueryResponse


class PartsClassifier:
    def __init__(self):
        self.scalar = None
        self.classifier = None

        # Get the data_filepath and test_filepath. We want things to break if
        # the appropriate params are not set
        data_filepath = rospy.get_param('~data_filepath')
        test_filepath = rospy.get_param('~test_filepath')

        # TODO: Train a classifier offline and save it. When running, we should
        # only be loading the classifier
        self.train_classifier(data_filepath, test_filepath)

        self.server = rospy.Service("rail_object_recognition/classify_parts", PartsQuery, self.classify)


    def classify(self, req):
        # classifications = []
        # descriptors = []

        X_test = np.array([item.descriptor for item in req.descriptors])
        if len(X_test.shape) == 1:
            X_test = X_test.reshape(1, -1)
        rospy.loginfo("Data Shape: {}".format(X_test.shape))
        classifications = self.classifier.predict_proba(X_test)
        rospy.loginfo("Classifications:\n{}".format(classifications))

        return PartsQueryResponse(classifications.reshape(-1).tolist())

    def train_classifier(self, data_filepath, test_filepath):
        np.random.seed(11)

        data = pandas.read_csv(data_filepath)
        # columns = data.columns.tolist()
        # columns_use = columns[:len(columns)-1]
        # data.drop(data.columns[len(data.columns)-1], axis=1, inplace=True)
        Y = data.Labels
        X = data.drop(['Objects','Labels'],axis=1)
        data_test = pandas.read_csv(test_filepath)
        X_test = data_test.drop(['Objects','Labels'],axis=1)
        Y_test = data_test.Labels
        rospy.loginfo("Data Shape: {}".format(X.shape))
        # X,Y = shuffle(X,Y,random_state=1)

        #data_test = pandas.read_csv(test_filepath)
        #X_test = data_test#.drop(['Objects'],axis=1)
        # rospy.loginfo(X_test.shape)

        ## Preprocessing
        #self.scalar = preprocessing.Standardscalar()
        #self.scalar.fit(X)
        #X = self.scalar.transform(X)
        self.classifier = KNeighborsClassifier(n_neighbors=8, weights='distance')
        self.classifier.fit(X,Y)
        rospy.loginfo("Classifier Accuracy: {}".format(self.classifier.score(X_test,Y_test)))
        Y_pred = self.classifier.predict(X_test)
        rospy.loginfo("Confusion Matrix:\n{}".format(confusion_matrix(Y_test, Y_pred)))

if __name__ == "__main__":
    rospy.init_node("parts_classifier_server")
    parts_classifier = PartsClassifier()
    rospy.spin()
