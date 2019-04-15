#!/usr/bin/env python

# ROS
import rospkg
import rospy

# numpy
import numpy

# scikit-learn
from sklearn import metrics
from sklearn.ensemble import AdaBoostClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import average_precision_score
from sklearn.metrics import precision_recall_curve
from sklearn.metrics import roc_auc_score
from sklearn.metrics import roc_curve
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import learning_curve
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle

# pyplot
import matplotlib.pyplot as pyplot


def evaluate_classifier():
    """Evaluate performance of different types of classifiers."""
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

    split = rospy.get_param('~split', 0.4)
    data_train, data_test, label_train, label_test = train_test_split(data, label, test_size=split)

    # TODO(enhancement): normalization is currently hardcoded; assumes a specific feature vector organization
    '''
    Potentially optional: normalize data
    Important for svm and nn...
    '''
    data_train_normalized = data_train
    data_train_normalized[:, 0] = data_train_normalized[:, 0]/50 - 1    # cielab L, normalizing to [-1,1]
    data_train_normalized[:, 1] /= 256                                  # cielab a, normalizing to [-1,1] (approximate)
    data_train_normalized[:, 2] /= 256                                  # cielab b, normalizing to [-1,1] (approximate)
    data_test_normalized = data_test
    data_test_normalized[:, 0] = data_test_normalized[:, 0]/50 - 1      # cielab L, normalizing to [-1,1]
    data_test_normalized[:, 1] /= 256                                   # cielab a, normalizing to [-1,1] (approximate)
    data_test_normalized[:, 2] /= 256                                   # cielab b, normalizing to [-1,1] (approximate)

    plot = rospy.get_param('~generate_plots', False)

    # containers for ROC and precision-recall plotting
    probs = dict()
    fpr = dict()
    tpr = dict()
    thresholds = dict()
    roc_label = dict()
    precision = dict()
    recall = dict()
    pr_label = dict()
    index = 0

    for classifier_type in types:
        print '\n----------------------------------------------------'
        print 'Evaluating classifier: ' + classifier_type

        classifier = prepare_classifier(classifier_type)

        print('Performing 10-fold cross validation...')
        if classifier_type in normalize_classifiers:
            scores = cross_val_score(classifier, data_normalized, label, cv=10)
        else:
            scores = cross_val_score(classifier, data, label, cv=10)
        print('Accuracy: %0.2f +/- %0.2f\n' % (scores.mean(), scores.std()))

        print('Detailed results on a %0.0f/%0.0f train/test split:' % ((1 - split)*100, split*100))
        if classifier_type in normalize_classifiers:
            classifier.fit(data_train_normalized, label_train)
            predicted = classifier.predict(data_test_normalized)
        else:
            classifier.fit(data_train, label_train)
            predicted = classifier.predict(data_test)
        print(metrics.classification_report(label_test, predicted))
        print(metrics.confusion_matrix(label_test, predicted))


        if plot:
            print('\nGenerating learning curve plot...')
            cross_val_size = 10
            step_size = data.shape[0]//20
            train_sizes = range(step_size, data.shape[0] - data.shape[0]//cross_val_size, step_size)
            if classifier_type in normalize_classifiers:
                train_sizes, train_scores, valid_scores = learning_curve(prepare_classifier(classifier_type),
                                                                         data_normalized, label,
                                                                         train_sizes=train_sizes, cv=cross_val_size)
            else:
                train_sizes, train_scores, valid_scores = learning_curve(prepare_classifier(classifier_type),
                                                                         data, label,
                                                                         train_sizes=train_sizes, cv=cross_val_size)
            y1_mean, y1_lower, y1_upper = calculate_means_with_bounds(train_scores)
            y2_mean, y2_lower, y2_upper = calculate_means_with_bounds(valid_scores)
            pyplot.figure()
            pyplot.ion()
            pyplot.fill_between(train_sizes, y1_lower, y1_upper, color='r', alpha=0.2)
            pyplot.plot(train_sizes, y1_mean, color='r', label='Training score')
            pyplot.fill_between(train_sizes, y2_lower, y2_upper, color='b', alpha=0.2)
            pyplot.plot(train_sizes, y2_mean, color='b', label='Cross-validation score')
            pyplot.axis([0, data.shape[0], 0.4, 1.05])
            pyplot.title(get_classifier_string(classifier_type))
            pyplot.xlabel('Training examples')
            pyplot.ylabel('Score')
            pyplot.legend(loc='lower right')
            pyplot.pause(0.05)

            # ROC curve
            if classifier_type in normalize_classifiers:
                probs[index] = classifier.predict_proba(data_test_normalized)[:, 1]
            else:
                probs[index] = classifier.predict_proba(data_test)[:, 1]
            fpr[index], tpr[index], thresholds[index] = roc_curve(label_test, probs[index])
            roc_label[index] = get_classifier_string(classifier_type) \
                + ('(area = %0.2f)' % roc_auc_score(label_test, probs[index]))

            # Precision-Recall curve
            precision[index], recall[index], _ = precision_recall_curve(label_test, probs[index])
            pr_label[index] = get_classifier_string(classifier_type) \
                + ('(area = %0.2f)' % average_precision_score(label_test, probs[index]))

            index += 1

    if plot:
        print('\n Plotting ROC curves...')
        pyplot.figure()
        pyplot.ion()
        for i in range(len(types)):
            pyplot.plot(fpr[i], tpr[i], label=roc_label[i])
        pyplot.xlabel('False positive rate')
        pyplot.ylabel('True positive rate')
        pyplot.title('ROC Curve')
        pyplot.legend(loc='lower right')
        pyplot.pause(0.05)

        print('\n Plotting Precision-Recall...')
        pyplot.figure()
        pyplot.ion()
        for i in range(len(types)):
            pyplot.plot(recall[i], precision[i], label=pr_label[i])
        pyplot.xlabel('Recall')
        pyplot.ylabel('Precision')
        pyplot.title('Precision-Recall Curve')
        pyplot.legend(loc='lower left')
        pyplot.pause(0.05)

        raw_input('Press [enter] to end program.  Note: this will close all plots!')


def prepare_classifier(classifier_type):
    """Return a classifier object for any supported classifier type."""
    # TODO(enhancement): classifier parameters are hardcoded
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


def calculate_means_with_bounds(score_list):
    """Calculate mean, lower bound (-1 stdev), and upper bound (+1 stdev) for a list of classification scores."""
    s_mean = []
    s_lower = []
    s_upper = []
    for scores in score_list:
        mean = numpy.mean(scores)
        stdev = numpy.std(scores)
        s_mean.append(mean)
        s_lower.append(mean - stdev)
        s_upper.append(mean + stdev)
    return s_mean, s_lower, s_upper


def get_classifier_string(classifier_type):
    """Convert the classifier_type parameter into a more human-readable string."""
    if classifier_type == 'decision_tree':
        return 'Decision Tree'
    elif classifier_type == 'random_forest':
        return 'Random Forest'
    elif classifier_type == 'ada_boost':
        return 'AdaBoost'
    elif classifier_type == 'knn':
        return 'KNN'
    elif classifier_type == 'svm':
        return 'SVM'
    elif classifier_type == 'logistic_regression':
        return 'Logistic Regression'
    elif classifier_type == 'nn1':
        return 'Neural Net (1 hidden layer)'
    elif classifier_type == 'nn2':
        return 'Neural Net (2 hidden layers)'


def load_data(filepath):
    """Parse training data and labels from a .csv file."""
    data = numpy.loadtxt(filepath, delimiter=',')
    x = data[:, :data.shape[1] - 1]
    y = data[:, data.shape[1] - 1]
    return x, y


if __name__ == '__main__':
    try:
        evaluate_classifier()
    except rospy.ROSInterruptException:
        pass
