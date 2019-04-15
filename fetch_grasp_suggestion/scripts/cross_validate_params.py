#!/usr/bin/env python

# ROS
import rospy
import rospkg

# numpy
import numpy

# scikit-learn
from sklearn.ensemble import AdaBoostClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import GridSearchCV
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle

# pyplot
import matplotlib.pyplot as pyplot


def cross_validate():
    """Perform cross-validation on given classifier types."""
    rospy.init_node('cross_validate_params')

    types = rospy.get_param('~classifier_types', 'decision_tree').split(',')
    supported_classifiers = ['decision_tree', 'random_forest', 'ada_boost', 'knn', 'svm', 'logistic_regression', 'nn1',
                             'nn2']

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

    # TODO(enhancement): normalization is currently hardcoded; assumes a specific feature vector organization
    '''
    Potentially optional: normalize data
    Important for svm and nn...
    '''
    '''
    data[:,0] = data[:,0]/50 - 1    # cielab L, normalizing to [-1,1]
    data[:,1] /= 256                # cielab a, normalizing to [-1,1] (approximate)
    data[:,2] /= 256                # cielab b, normalizing to [-1,1] (approximate)
    '''

    data, label = shuffle(data, label)

    plot = rospy.get_param('~generate_plots', False)

    for classifier_type in types:
        print '\n----------------------------------------------------'
        print 'Performing cross-validation for ' + classifier_type

        parameters = prepare_parameter_grid(classifier_type)
        classifier = GridSearchCV(prepare_classifier(classifier_type), parameters, cv=5)
        classifier.fit(data, label)

        means = classifier.cv_results_['mean_test_score']
        stds = classifier.cv_results_['std_test_score']
        print("\nDetailed breakdown:")
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            print("%0.3f +/-%0.03f for %r" % (mean, std*2, params))

        print("\nBest params:")
        print(classifier.best_params_)

        if plot:
            pyplot.figure()
            pyplot.ion()
            make_plot(classifier_type, means, stds, classifier)
            pyplot.pause(0.05)

    if plot:
        raw_input('Press [enter] to end program.  Note: this will close all plots!')


def prepare_classifier(classifier_type):
    """Return a classifier object for any supported classifier type."""
    if classifier_type == 'decision_tree':
        return DecisionTreeClassifier()
    elif classifier_type == 'random_forest':
        return RandomForestClassifier()
    elif classifier_type == 'ada_boost':
        return AdaBoostClassifier()
    elif classifier_type == 'knn':
        return KNeighborsClassifier()
    elif classifier_type == 'svm':
        return SVC()
    elif classifier_type == 'logistic_regression':
        return LogisticRegression()
    elif classifier_type == 'nn1':
        return MLPClassifier()
    elif classifier_type == 'nn2':
        return MLPClassifier()


def prepare_parameter_grid(classifier_type):
    """Return a parameter grid for cross-validation of any supported classifier type."""
    # TODO(enhancement): grid parameters are hardcoded for each classifier
    if classifier_type == 'decision_tree':
        return [{'criterion': ['gini', 'entropy'], 'min_samples_leaf': [1, 2, 4, 8],
                 'min_samples_split': [2, 4, 8, 16]}]
    elif classifier_type == 'random_forest':
        return [{'n_estimators': [10, 20, 50, 100], 'criterion': ['gini', 'entropy'],
                 'max_features': ['sqrt', 'log2', None]}]
    elif classifier_type == 'ada_boost':
        return [{'n_estimators': [25, 50, 100, 200], 'learning_rate': [.01, .1, 1, 10, 100]}]
    elif classifier_type == 'knn':
        return [{'n_neighbors': [1, 2, 5, 10, 20, 50], 'weights': ['uniform', 'distance']}]
    elif classifier_type == 'svm':
        return [{'C': [0.1, 1.0, 10.0], 'kernel': ['rbf', 'linear', 'sigmoid'],
                 'decision_function_shape': ['ovo', 'ovr']},
                {'C': [0.1, 1.0, 10.0], 'kernel': ['poly'], 'degree': [3], 'decision_function_shape': ['ovo', 'ovr']}]
    elif classifier_type == 'logistic_regression':
        return [{'penalty': ['l1', 'l2'], 'C': [.01, 0.1, 1.0, 10.0], 'solver': ['liblinear']},
                {'penalty': ['l2'], 'C': [.01, 0.1, 1.0, 10.0], 'solver': ['lbfgs']}]
    elif classifier_type == 'nn1':
        return [{'hidden_layer_sizes': [10, 50, 100], 'learning_rate': ['constant', 'invscaling', 'adaptive'],
                 'max_iter': [200, 500, 1000]}]
    elif classifier_type == 'nn2':
        return [{'hidden_layer_sizes': [(10, 10), (10, 50), (10, 100), (50, 10), (50, 50), (50, 100), (100, 10),
                                        (100, 50), (100, 100)],
                 'learning_rate': ['constant', 'invscaling', 'adaptive'], 'max_iter': [200, 500, 1000]}]


def make_plot(classifier_type, means, stds, classifier):
    """Make a scatter plot of cross-validation scores for any supported classifier type."""
    # TODO(cleanup): plotting procedure is hardcoded for each classifier type
    if classifier_type == 'decision_tree':
        x = [2, 4, 8, 16]
        y_gini = [[], [], [], []]
        y_entropy = [[], [], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = x.index(params['min_samples_split'])
            if params['min_samples_leaf'] == 2:
                i = 1
            elif params['min_samples_leaf'] == 4:
                i = 2
            elif params['min_samples_leaf'] == 8:
                i = 3

            if params['criterion'] == 'gini':
                y_gini[i].insert(j, mean)
            else:
                y_entropy[i].insert(j, mean)
        pyplot.plot(x, y_gini[0], 'bo', x, y_entropy[0], 'ro',
                    x, y_gini[1], 'b+', x, y_entropy[1], 'r+',
                    x, y_gini[2], 'bx', x, y_entropy[2], 'rx',
                    x, y_gini[3], 'bv', x, y_entropy[3], 'rv')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 18, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('min_samples_split')
        pyplot.ylabel('mean score')
    elif classifier_type == 'random_forest':
        x = [10, 20, 50, 100]
        y_gini = [[], [], []]
        y_entropy = [[], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = x.index(params['n_estimators'])
            if params['max_features'] == 'log2':
                i = 1
            elif params['max_features'] is None:
                i = 2

            if params['criterion'] == 'gini':
                y_gini[i].insert(j, mean)
            else:
                y_entropy[i].insert(j, mean)
        pyplot.plot(x, y_gini[0], 'b+', x, y_entropy[0], 'r+',
                    x, y_gini[1], 'bx', x, y_entropy[1], 'rx',
                    x, y_gini[2], 'bo', x, y_entropy[2], 'ro')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 110, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('n_estimators')
        pyplot.ylabel('mean score')
    elif classifier_type == 'ada_boost':
        x = [25, 50, 100, 200]
        y = [[], [], [], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = x.index(params['n_estimators'])
            if params['learning_rate'] == .1:
                i = 1
            elif params['learning_rate'] == 1:
                i = 2
            elif params['learning_rate'] == 10:
                i = 3
            elif params['learning_rate'] == 100:
                i = 4
            y[i].insert(j, mean)
        pyplot.plot(x, y[0], 'bo',
                    x, y[1], 'ro',
                    x, y[2], 'co',
                    x, y[3], 'yo',
                    x, y[4], 'ko')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 225, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('n_estimators')
        pyplot.ylabel('mean score')
    elif classifier_type == 'knn':
        x = [1, 2, 5, 10, 20, 50]
        y = [[], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = x.index(params['n_neighbors'])
            if params['weights'] == 'distance':
                i = 1
            y[i].insert(j, mean)
        pyplot.plot(x, y[0], 'bo',
                    x, y[1], 'ro')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 51, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('n_estimators')
        pyplot.ylabel('mean score')
    elif classifier_type == 'svm':
        # x = ['linear','poly', 'sigmoid', 'rbf']
        x = [1, 2, 3, 4]
        y_ovo = [[], [], []]
        y_ovr = [[], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = 0
            if params['C'] == 1.0:
                i = 1
            elif params['C'] == 10.0:
                i = 2
            if params['kernel'] == 'poly':
                j = 1
            elif params['kernel'] == 'sigmoid':
                j = 2
            elif params['kernel'] == 'rbf':
                j = 3

            if params['decision_function_shape'] == 'ovo':
                y_ovo[i].insert(j, mean)
            else:
                y_ovr[i].insert(j, mean)
        pyplot.plot(x, y_ovo[0], 'b+', x, y_ovr[0], 'r+',
                    x, y_ovo[1], 'bx', x, y_ovr[1], 'rx',
                    x, y_ovo[2], 'bo', x, y_ovr[2], 'ro')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 5, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('kernel : [linear, poly, sigmoid, rbf]')
        pyplot.ylabel('mean score')
    elif classifier_type == 'logistic_regression':
        x = [0.01, .1, 1.0, 10.0]
        yliblinear = [[], []]
        ylbfgs = []
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            j = x.index(params['C'])
            if params['penalty'] == 'l2':
                i = 1

            if params['solver'] == 'liblinear':
                yliblinear[i].insert(j, mean)
            else:
                ylbfgs.insert(j, mean)
        pyplot.plot(x, yliblinear[0], 'b+',
                    x, yliblinear[1], 'bx', x, ylbfgs, 'rx')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 11, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('C')
        pyplot.ylabel('mean score')
    elif classifier_type == 'nn1':
        ind = [10, 50, 100]
        x = [1, 2, 3]
        y_constant = [[], [], []]
        y_invscaling = [[], [], []]
        y_adaptive = [[], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            if params['max_iter'] == 500:
                i = 1
            elif params['max_iter'] == 1000:
                i = 2
            j = ind.index(params['hidden_layer_sizes'])

            if params['learning_rate'] == 'constant':
                y_constant[i].insert(j, mean)
            elif params['learning_rate'] == 'invscaling':
                y_invscaling[i].insert(j, mean)
            else:
                y_adaptive[i].insert(j, mean)
        pyplot.plot(x, y_constant[0], 'b+', x, y_invscaling[0], 'r+', x, y_adaptive[0], 'y+',
                    x, y_constant[1], 'bx', x, y_invscaling[1], 'rx', x, y_adaptive[1], 'yx',
                    x, y_constant[2], 'bo', x, y_invscaling[2], 'ro', x, y_adaptive[2], 'yo')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 4, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('hidden layer size : [10, 50, 100]')
        pyplot.ylabel('mean score')
    elif classifier_type == 'nn2':
        ind = [(10, 10), (10, 50), (10, 100), (50, 10), (50, 50), (50, 100), (100, 10), (100, 50), (100, 100)]
        x = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        y_constant = [[], [], []]
        y_invscaling = [[], [], []]
        y_adaptive = [[], [], []]
        for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
            i = 0
            if params['max_iter'] == 500:
                i = 1
            elif params['max_iter'] == 1000:
                i = 2
            j = ind.index(params['hidden_layer_sizes'])

            if params['learning_rate'] == 'constant':
                y_constant[i].insert(j, mean)
            elif params['learning_rate'] == 'invscaling':
                y_invscaling[i].insert(j, mean)
            else:
                y_adaptive[i].insert(j, mean)
        pyplot.plot(x, y_constant[0], 'b+', x, y_invscaling[0], 'r+', x, y_adaptive[0], 'y+',
                    x, y_constant[1], 'bx', x, y_invscaling[1], 'rx', x, y_adaptive[1], 'yx',
                    x, y_constant[2], 'bo', x, y_invscaling[2], 'ro', x, y_adaptive[2], 'yo')
        auto_axis = pyplot.axis()
        pyplot.axis([0, 10, auto_axis[2], auto_axis[3]])
        pyplot.xlabel('hidden layer size : [(10,10),(10,50),(10,100),(50,10),(50,50),(50,100),(100,10),(100,50),'
                      '(100,100)]')
        pyplot.ylabel('mean score')


def load_data(filename):
    """Parse training data and labels from a .csv file."""
    data = numpy.loadtxt(filename, delimiter=',')
    x = data[:, :data.shape[1] - 1]
    y = data[:, data.shape[1] - 1]
    return x, y


if __name__ == '__main__':
    try:
        cross_validate()
    except rospy.ROSInterruptException:
        pass
