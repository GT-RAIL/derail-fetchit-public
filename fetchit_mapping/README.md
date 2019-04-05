# FetchIt Challenge Mapping, Localization, and Navigation Pipline

This repository contains the pipeline to do 2D/3D mapping for FetchIt Challenge.

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
2. After the robot finishes one 360 degree rotation and stops moving, the 2D and 3D maps should
be saved in the `/maps` folder of this package.
3. There is a chance the robot never begins moving before the mapping pipeline, causing it never
rotate and only make a partial map. In that case, just re-run the launch (after stopping the 
previous).

## Localize with 2D maps (and publish 3D collision map)
1. Run `roslaunch fetchit_mapping fetchit_localization.launch`.
2. After launch, give the robot an initial pose estimate through the rviz GUI or teleop the
robot to allow it to localize.
3. Once localized, `base_link` transform estimate should remain accurate.
4. **Note** that localization exclusively relies on the 2D map while the 3D octomap for static
obstacle collision checking uses the transforms calculated from 2D localization.

## Waypoint navigation
After completing the previous steps, 
1. Run `roslaunch fetchit_mapping fetchit_navigation.launch`. 
2. Run `rosrun fetchit_mapping fetch_goal_publisher`. Input waypoints as x, y, theta (degrees). 
Example: 0.9 0.1 0 for a location near Schunk machine (in simulation).

## Notes on launch parameters
1. `sim` used to toggle whether to start a simulator.
2. `dynamic_env` used to toggle dynamic obstacle filtering.
3. `collision_mapping` used to toggle 3d collision mapping.
4. `2d_map` used to name output 2d map for localization.
5. `3d_map` used to name output 3d map for collisions.

## To Dos
1. Fix yaml/no yaml naming of maps across nav and mapping
1. Automatic localization motion... any ideas?
1. Testing for localization/mapping.
1. Better mapping motion to improve initial map?
