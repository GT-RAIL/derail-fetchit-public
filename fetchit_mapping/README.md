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

## Navigation - PID 

The navigation server used is based on a [unicycle model] where only the angular velocity is controlled. 
Navigating from one waypoint to another is broken down into three stages: 
- an in-place rotation to face the destination. 
- move towards destination with constant linear velocity, PID controlled angular velocity. 
- an in-place rotation to face in required direction at destination. 

The rotations in all three steps are governed by the PID coefficients used to control the angular velocity. 
It is very important to use good PID coefficients to avoid overshoot, oscillations, colliding into obstacles, spinning around, etc. Some issues that have been noticed as a consequence of using bad coefficients are: 
- Initial velocity is too low for the robot to start moving. Weird things happen after this - robot waits for a long time and then suddenly goes to a different location. 
- Robot oscillates too much in transit that it collides slightly with the tables or other obstacles. 
- Robot overshoots at destination resulting in an oscillatory/circular motion around the destination. 

### Tuning the PID coeffiencients:

For reasons highlighted above, it would be necessary to tune the coefficients on the robot before the challenge. While very careful tuning can take long, one should be able to get good coefficients within 15 minutes (it is recommended to try it a few times before the challenge to see how long it takes). Since we do not use any formal methods, there are no rigid rules on what procedure to follow for tuning. The one I used is: 

- Increase the K_P value with K_D and K_I set to zero until the robot oscillates, then use half of this value. In the worst case, using zero values for K_I and K_D with this value for K_P should also produce decent results. 
- Set K_I to a very small value (say about 0.001) and then increase it until the robot oscillates. 
- Then finally reduce the oscillations by reducing K_I or increasing K_d. 
- Ensure that K_I and K_d values that are finally used are very small. The values that are currently used should provide a good reference. 

Here are a few more references to tuning PID controllers:
- https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
- https://www.autonomousrobotslab.com/pid-control.html

## Reposition - PID 

Unlike navigation, reposition server controls the linear velocity along with the angular velocity. This makes it capable of moving backwards. 
Consider using the same PID values that were derived for navigation server for the angular velocity control coefficients (K_pw, K_iw, and K_dw) for reposition server. 

In addition to this it would be necessary to tune the K_p, K_i and K_d values for the linear velocity control of the reposition server. Starting from the same values that were used for angular velocity control, repeat the procedure outlined in navigation server's PID tuning for this. It is not necessary to spend much time here. A simple proportional controller (K_d = 0, K-i = 0) should work. Main parameters to look out for are: the velocity of the robot when it translates, and whether there are oscillations. 
Something that has been noticed is that while some coefficients work well for forward motion, the robot does not start using these coefficients for backward motion (especially when distance to goal is less). This is probably because the robot takes more force to move backwards than forward. Ensure that the coefficients being used can get the robot moving well in both directions for all different distances to goal. 

The parameters are specified in the navigation launch files (Angular P,I,D gains and linear P,I,D gains). 


[unicycle model]: http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
