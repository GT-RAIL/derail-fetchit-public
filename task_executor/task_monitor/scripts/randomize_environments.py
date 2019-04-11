#!/usr/bin/env python
# Randomize the test environment and then launch

from __future__ import print_function, division

import os
import sys
import argparse
import subprocess

import numpy as np


# The bulk of the script. TODO: Make this a bit smarter

AVAILABLE_ROSLAUNCH_COMMANDS = [
    # Base stalled
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=true base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Base moved back
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=true base_stalled:=false base_collided:=false incorrect_map:=false hardware_failure:=false gui:=false",

    # Incorrect localization
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=false incorrect_map:=true hardware_failure:=false gui:=false",

    # Base collided
    "roslaunch fetch_gazebo task_worlds.launch object_location_idx:=0 pickup_distraction:=none place_distraction:=none door_blocked:=false door_block_invisible:=false head_moved:=false base_moved_back:=false base_stalled:=false base_collided:=true incorrect_map:=false hardware_failure:=false gui:=false"
]

def main(*args, **kwargs):
    process = subprocess.Popen(np.random.choice(AVAILABLE_ROSLAUNCH_COMMANDS), shell=True)
    process.wait()


if __name__ == '__main__':
    main()
