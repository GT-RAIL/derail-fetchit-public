#!/usr/bin/env python

# ROS
from geometry_msgs.msg import PoseArray
from fetch_grasp_suggestion.srv import ClassifyAll
from fetch_grasp_suggestion.srv import ClassifyGraspPair
from math import sqrt
import matplotlib.pyplot as pyplot
import numpy as np
import rospy
import rospkg

# scikit-learn
from sklearn.externals import joblib

# tree interpreter (package for interpreting scikit-learn decision trees and random forests
from treeinterpreter import treeinterpreter as ti


class ClassifierNode:

    def __init__(self):
        """Initialize classifier service in a ROS node."""
        filepath = rospy.get_param('~file_name', 'decision_tree.pkl')
        if len(filepath) > 0 and filepath[0] != '/':
            filepath = rospkg.RosPack().get_path('fetch_grasp_suggestion') + '/data/classifier/' + filepath

        self.model = joblib.load(filepath)

        jobs = rospy.get_param('~n_jobs', 1)
        if 'n_jobs' in self.model.get_params().keys():
            self.model.set_params(n_jobs=jobs)

        # Note: Interpretation report is only implemented for decision trees and random forests!
        self.interpret_trees = rospy.get_param('~interpret_trees', False)

        self.object_feature_size = rospy.get_param('~object_feature_size', 6)

        self.service = rospy.Service('classify_grasp_pair', ClassifyGraspPair, self.classify)
        self.classify_all_service = rospy.Service('classify_all', ClassifyAll, self.classify_all)

    def classify(self, req):
        """Return binary classification of an ordered grasp pair feature vector."""
        return self.model.predict(np.asarray(req.feature_vector).reshape(1, -1))

    def classify_all(self, req):
        """Return ranked list of grasps from binary classification of all pairs of grasp feature vectors"""
        rank_counts = [0, ]*len(req.grasp_list.grasps)
        data = np.asarray([[]])
        data_init = False
        index_transform = dict()
        for i in range(len(req.grasp_list.grasps) - 1):
            for j in range(i + 1, len(req.grasp_list.grasps)):
                # set up feature vectors
                if len(req.object_features) > 0:
                    feature_vector_ij = list(req.object_features)
                    feature_vector_ji = feature_vector_ij
                    features_i = np.asarray(req.grasp_list.grasps[i].heuristics)
                    features_j = np.asarray(req.grasp_list.grasps[j].heuristics)
                else:
                    # alternate method of encoding object features in heuristic array, only used if req.object_features
                    # is left empty, which assumes the object features are instead prepended to each grasp's heuristics
                    feature_vector_ij = req.grasp_list.grasps[i].heuristics[:self.object_feature_size]
                    feature_vector_ji = feature_vector_ij
                    features_i = np.asarray(req.grasp_list.grasps[i].heuristics[self.object_feature_size:])
                    features_j = np.asarray(req.grasp_list.grasps[j].heuristics[self.object_feature_size:])
                feature_vector_ij = np.append(feature_vector_ij, features_i - features_j).reshape(1, -1)
                feature_vector_ji = np.append(feature_vector_ji, features_j - features_i).reshape(1, -1)

                if not data_init:
                    data = feature_vector_ij
                    data = np.concatenate((data, feature_vector_ji), axis=0)
                    data_init = True
                else:
                    data = np.concatenate((data, feature_vector_ij), axis=0)
                    data = np.concatenate((data, feature_vector_ji), axis=0)

                index_transform[(i, j)] = data.shape[0] - 2
                index_transform[(j, i)] = data.shape[0] - 1

        predictions = self.model.predict(data)

        for i in range(len(req.grasp_list.grasps) - 1):
            for j in range(i + 1, len(req.grasp_list.grasps)):

                # classify
                if predictions[index_transform[(i, j)]] == 1:
                    rank_counts[i] += 1

                if predictions[index_transform[(j, i)]] == 1:
                    rank_counts[j] += 1

        # sort by best rank count
        result = PoseArray()
        result.header.frame_id = req.grasp_list.grasps[0].pose.header.frame_id
        sorted_indices = np.flipud(np.argsort(rank_counts))  # use this for all pairs method
        for i in sorted_indices:
            result.poses.append(req.grasp_list.grasps[i].pose.pose)

        # optional interpretation report (only for decision tree and random forest models!)
        if self.interpret_trees and len(result.poses) > 0:
            prediction, bias, contributions = ti.predict(self.model, data)
            contributions_false = [[] for i in range(data.shape[1])]
            contributions_true = [[] for i in range(data.shape[1])]
            contributions_all = [[] for i in range(data.shape[1])]
            contributions_best = [[] for i in range(data.shape[1])]
            best_index = sorted_indices[0]
            for i in range(data.shape[0]):
                if prediction[i][0] > 0.5:
                    for j in range(len(contributions[i])):
                        contributions_false[j].append(abs(contributions[i][j][0]))
                        contributions_all[j].append(abs(contributions[i][j][0]))
                else:
                    for j in range(len(contributions[i])):
                        contributions_true[j].append(abs(contributions[i][j][1]))
                        contributions_all[j].append(abs(contributions[i][j][1]))
            for i in range(len(req.grasp_list.grasps)):
                if i != best_index:
                    k = index_transform[(best_index, i)]
                    if prediction[k][1] > 0.5:
                        for j in range(len(contributions[k])):
                            contributions_best[j].append(abs(contributions[k][j][1]))

            means_false = np.mean(contributions_false, axis=1)
            stds_false = np.std(contributions_false, axis=1)
            sterr_false = stds_false / sqrt(len(contributions_false[0]))
            means_true = np.mean(contributions_true, axis=1)
            stds_true = np.std(contributions_true, axis=1)
            sterr_true = stds_true / sqrt(len(contributions_true[0]))
            means_all = np.mean(contributions_all, axis=1)
            stds_all = np.std(contributions_all, axis=1)
            sterr_all = stds_all / sqrt(len(contributions_all[0]))
            means_best = np.mean(contributions_best, axis=1)
            stds_best = np.std(contributions_best, axis=1)
            sterr_best = stds_best / sqrt(len(contributions_best[0]))

            fig, ax = pyplot.subplots()
            index = np.arange(data.shape[1])
            bar_width = 0.35
            opacity = 0.4
            error_config = {'ecolor': '0.3'}

            rects1 = pyplot.bar(index, means_true, bar_width,
                                alpha=opacity,
                                color='b',
                                yerr=sterr_true,
                                error_kw=error_config,
                                label='True')

            rects2 = pyplot.bar(index + bar_width, means_false, bar_width,
                                alpha=opacity,
                                color='r',
                                yerr=sterr_false,
                                error_kw=error_config,
                                label='False')

            pyplot.xlabel('Feature')
            pyplot.ylabel('Contribution (magnitude)')
            pyplot.title('Contributions per feature for true and false classifications')
            pyplot.xticks(index + bar_width / 2, ('l', 'a', 'b', 'z', 'x', 'y', 'h1', 'h2', 'h3', 'h4', 'h5'))
            pyplot.legend()

            pyplot.tight_layout()

            pyplot.figure()
            pyplot.bar(index, means_all, align='center', alpha=0.4, yerr=sterr_all, error_kw=error_config)
            pyplot.xticks(index, ('l', 'a', 'b', 'z', 'x', 'y', 'h1', 'h2', 'h3', 'h4', 'h5'))
            pyplot.xlabel('Feature')
            pyplot.ylabel('Contribution (magnitude)')
            pyplot.title('Contributions per feature for all classification decisions')

            pyplot.figure()
            pyplot.bar(index, means_best, align='center', alpha=0.4, yerr=stds_best, error_kw=error_config)
            pyplot.xticks(index, ('l', 'a', 'b', 'z', 'x', 'y', 'h1', 'h2', 'h3', 'h4', 'h5'))
            pyplot.xlabel('Feature')
            pyplot.ylabel('Contribution (magnitude)')
            pyplot.title('Contributions per feature for all pairs including the top-rated grasp')

            pyplot.show()

            print 'returning from service call...'

        return result


if __name__ == '__main__':
    rospy.init_node('classifier_node')

    classifier_node = ClassifierNode()
    print 'Ready to classify.'

    rospy.spin()
