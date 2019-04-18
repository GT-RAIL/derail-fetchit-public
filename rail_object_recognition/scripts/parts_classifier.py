#!/usr/bin/env python

from rail_segmentation.srv import *
import rospy


from sklearn import preprocessing
from sklearn.metrics import confusion_matrix
from sklearn.neighbors import KNeighborsClassifier
from sklearn.utils import shuffle
from sklearn.externals import joblib
import numpy as np
import pandas
import random

class PartsClassifier:
	def __init__(self):
		rospy.init_node("parts_classifier_server")
		self.scalar = None
		self.classifier = None
		self.descriptor_list = []
		self.classes = []
		self.train_classifier()
		self.server = rospy.Service("rail_segmentation/classify_parts", PartsQuery, self.classify)
		rospy.spin()


	def classify(self, req):
		for item in req.descriptors:
			X_test = item.descriptor
			print("item descriptor")
			
			'''with open("/home/nithin/Desktop/data/testing.csv", "a") as fh:
				desc = item.descriptor
				desc = [str(num) for num in desc]
				desc = ", ".join(desc)
				fh.write(label)
				fh.write(",")
				fh.write(desc + "\n")'''
			##print(item.descriptor)
			self.descriptor_list.append(X_test)
			X_test = np.array(X_test)
			X_test = X_test.reshape(1,X_test.shape[0])

			# Not finish yet
			# X_test = self.scalar.transform(X_test)
			rospy.loginfo_once(self.classifier.predict_proba(X_test))
			self.classes.append(self.classifier.predict_proba(X_test))
			
		return self.classes

	def train_classifier(self):
		np.random.seed(11)

		#filename = '/home/nithin/Desktop/Objects/objects_fetchit_1.csv'
		filename_test = '/home/nithin/Desktop/Objects/objects_fetchit_1.csv'
		filename = '/home/nithin/Desktop/data/dataset.csv'
		data = pandas.read_csv(filename)
		# columns = data.columns.tolist()
		# columns_use = columns[:len(columns)-1]
		# data.drop(data.columns[len(data.columns)-1], axis=1, inplace=True)
		Y = data.Labels
		X = data.drop(['Objects','Labels'],axis=1)
		data_test = pandas.read_csv(filename_test)
		X_test = data_test.drop(['Objects','Labels'],axis=1)
		Y_test = data_test.Labels
		print(X.shape)
		# X,Y = shuffle(X,Y,random_state=1)

		#filename_test = '/home/nithin/Desktop/Objects/object_0.csv'
		#data_test = pandas.read_csv(filename_test)
		#X_test = data_test#.drop(['Objects'],axis=1)
		# print(X_test.shape)

		## Preprocessing
		#self.scalar = preprocessing.Standardscalar()
		#self.scalar.fit(X)
		#X = self.scalar.transform(X)
		self.classifier = KNeighborsClassifier(n_neighbors=8, weights='distance')
		self.classifier.fit(X,Y)
		print("Accuracy:")
		print(self.classifier.score(X_test,Y_test))
		Y_pred = self.classifier.predict(X_test)
		print(confusion_matrix(Y_test, Y_pred))

if __name__ == "__main__":
	parts_classifier = PartsClassifier()
