# FetchIt Challenge Mapping/Localization Pipline

This repository contains the pipeline to do 2D/3D mapping for FetchIt Challenge.

Current Status: builds and runs 2D/3D mapping. See ToDos for what's missing.

## Install
1. Depends on special "RAIL" version of challenge world. Available in `rail_test_world` folder.
You need to move the files in that folder to the appropriate locations within the 
`fetchit_challenge` package first. See README in `rail_test_world`.
2. Clone the repo, and catkin_make. Assumes you have top-level `fetch_gazebo`, `fetch_ros`, 
and `fetchit_challenge` installed.
3. Install all **ROS** `octomap` packages from [here](https://github.com/OctoMap) (actual 
`octomap` repo/source build not necessary).. Make sure this includes rviz plugins too.

## Get initial arena map
1. Run `roslaunch fetchit_mapping get_initial_map.launch`.
2. Save maps by ...to be continued...

## To Dos
1. Localization nodes.
1. Filter points outside of the 5 foot radius to account for dynamic obstacles.
1. Tune map resolution sizes.
1. Tune rotation/tilt speeds.
1. Moving within map to get better map?