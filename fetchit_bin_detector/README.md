# FetchIt Challenge Bin Pose Detection Pipeline

This repository contains the pipeline to detect bin poses.

## Install 

## Get Bin Poses
1. Run `roslaunch fetchit_bin_detector detector_demo.launch`.
1. Run `rosrun fetchit_bin_detector bin_detector_node`.

## Coordinate Frame Convention
1. I have made the x-axis align with the small wall, y-axis align with the handle, and z-axis vertical
as shown in the images below:
[logo]: https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Logo Title Text 2"

## To Dos
1. Finish README.
1. Handle occluded 2nd wall case.
1. Handle upside-down z case.
1. Testing tables and base.