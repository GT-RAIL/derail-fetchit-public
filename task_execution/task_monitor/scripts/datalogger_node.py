#!/usr/bin/env python
# Run bag processes in the background and create new segments on every service
# call to the bagging process

from __future__ import print_function, division

import os
import sys
import copy
import subprocess

from ruamel.yaml import YAML

import rospy
import rospkg

from std_srvs.srv import Empty, EmptyResponse


# YAML init

yaml = YAML(typ='safe')


# The class to perform the bagging

class DataLogger(object):
    """Logs data specified by the YAML file"""

    CONFIG_FILENAME = os.path.join(
        rospkg.RosPack().get_path('task_monitor'),
        'config/datalogger.yaml'
    )
    DATA_DIRECTORY = os.path.join(
        rospkg.RosPack().get_path('task_monitor'),
        'data',
        '2019-04-10'
        # NOTE: Make sure to update the above to the desired subfolder / symlink
    )

    ROSBAG_CMD = [
        'rosbag', 'record',
        '--duration=10m',  # Automatically split every 10m
        '--split',
        '-o', 'fetch-deliver',
        '-b', "0",
        '--chunksize=1024',
        '--lz4',
        '__name:=datalogger_record'
    ]
    ROSBAG_KILL_CMD = ['rosnode', 'kill', 'datalogger_record']

    START_SERVICE_NAME = '~start'
    STOP_SERVICE_NAME = '~stop'
    SPLIT_SERVICE_NAME = '~split'

    def __init__(self):
        # The config
        self.config = None

        # The process to do the bagging
        self._bag_process = None

        # The services to start, stop, and split bagging
        self._start_service = rospy.Service(DataLogger.START_SERVICE_NAME, Empty, self._start_srv)
        self._stop_service = rospy.Service(DataLogger.STOP_SERVICE_NAME, Empty, self._stop_srv)
        self._split_service = rospy.Service(DataLogger.SPLIT_SERVICE_NAME, Empty, self._split_srv)

        # Ready!
        rospy.loginfo("datalogger node is ready...")

    def _start_srv(self, req):
        self.start()
        return EmptyResponse()

    def _stop_srv(self, req):
        self.stop()
        return EmptyResponse()

    def _split_srv(self, req):
        self.split()
        return EmptyResponse()

    def start(self):
        if self._bag_process is not None:
            return

        cmd = copy.copy(DataLogger.ROSBAG_CMD)
        with open(DataLogger.CONFIG_FILENAME, 'r') as fd:
            self.config = yaml.load(fd)

        # Update the cmd
        if len(self.config['include_regex']) > 0:
            included_topics = "|".join(self.config['include_regex'])
            print(included_topics)
            cmd.append('-e')
            cmd.append('{}'.format(included_topics))

        if len(self.config['node']) > 0:
            cmd.append("--node={}".format(self.config['node']))

        if len(self.config['exclude_regex']) > 0:
            excluded_topics = "|".join(self.config['exclude_regex'])
            print(excluded_topics)
            cmd.append('-x')
            cmd.append('{}'.format(excluded_topics))

        print(cmd)

        # Open the process
        self._bag_process = subprocess.Popen(
            cmd,
            cwd=DataLogger.DATA_DIRECTORY,
            preexec_fn=os.setpgrp
        )

    def stop(self):
        if self._bag_process is not None:
            subprocess.check_call(DataLogger.ROSBAG_KILL_CMD)
            assert self._bag_process.wait() == 0, "Error in record process"
        self._bag_process = None

    def split(self):
        self.stop()
        self.start()


# The main

if __name__ == '__main__':
    rospy.init_node('datalogger')
    datalogger = DataLogger()
    rospy.spin()
