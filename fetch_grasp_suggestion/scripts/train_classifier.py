#!/usr/bin/env python

# ROS
import rospy
import rospkg

# numpy
import numpy

# scikit-learn
from sklearn.ensemble import AdaBoostClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle


def train_classifier():
    """Train a classifier and save it as a .pkl for later use."""
    rospy.init_node('train_classifier')

    types = rospy.get_param('~classifier_types', 'decision_tree').split(',')
    supported_classifiers = ['decision_tree', 'random_forest', 'ada_boost', 'knn', 'svm', 'logistic_regression', 'nn1',
                             'nn2']
    normalize_classifiers = ['knn', 'nn1', 'nn2']

    if 'all' in types:
        types = supported_classifiers

    for classifier_type in types:
        if classifier_type not in supported_classifiers:
            usage = 'Unsupported classifier type: ' + classifier_type + '. Supported classifiers are:'
            for classifier_string in supported_classifiers:
                usage += '\n\t' + classifier_string
            print usage
            return

    filepath = rospy.get_param('~file_name', 'grasp_data.csv')
    if len(filepath) > 0 and filepath[0] != '/':
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('fetch_grasp_suggestion') + '/data/grasp_preferences/' + filepath

    data, label = load_data(filepath)
    print '\nImported', data.shape[0], 'training instances'

    data, label = shuffle(data, label)

    # TODO(enhancement): normalization is currently hardcoded; assumes a specific feature vector organization
    '''
    Potentially optional: normalize data
    Important for svm and nn...
    '''
    data_normalized = data
    data_normalized[:, 0] = data_normalized[:, 0]/50 - 1    # cielab L, normalizing to [-1,1]
    data_normalized[:, 1] /= 256                            # cielab a, normalizing to [-1,1] (approximate)
    data_normalized[:, 2] /= 256                            # cielab b, normalizing to [-1,1] (approximate)

    for classifier_type in types:
        print '\n----------------------------------------------------'
        print 'Training classifier: ' + classifier_type

        classifier = prepare_classifier(classifier_type)

        print('Training on the full dataset...')
        if classifier_type in normalize_classifiers:
            classifier.fit(data_normalized, label)
        else:
            classifier.fit(data, label)

        joblib.dump(classifier, classifier_type + '.pkl')
        print('Saved model ' + classifier_type + '.pkl to current directory.')


def prepare_classifier(classifier_type):
    """Return a classifier object for any supported classifier type."""
    # Note: values below were chosen from analysis of cross validation
    if classifier_type == 'decision_tree':
        return DecisionTreeClassifier()
    elif classifier_type == 'random_forest':
        return RandomForestClassifier(n_estimators=50)
    elif classifier_type == 'ada_boost':
        return AdaBoostClassifier(n_estimators=50)
    elif classifier_type == 'knn':
        return KNeighborsClassifier(weights='distance')
    elif classifier_type == 'svm':
        return SVC(C=10, probability=True)
    elif classifier_type == 'logistic_regression':
        return LogisticRegression()
    elif classifier_type == 'nn1':
        return MLPClassifier(learning_rate='adaptive', max_iter=500)
    elif classifier_type == 'nn2':
        return MLPClassifier(learning_rate='adaptive', hidden_layer_sizes=(100, 50), max_iter=1000)


def load_data(filepath):
    """Parse training data and labels from a .csv file."""
    data = numpy.loadtxt(filepath, delimiter=',')
    x = data[:, :data.shape[1] - 1]
    y = data[:, data.shape[1] - 1]
    return x, y


if __name__ == '__main__':
    try:
        train_classifier()
    except rospy.ROSInterruptException:
        pass
