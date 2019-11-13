# rail_part_affordance_detection
This is a ROS wrapper for [AffordanceNet](https://github.com/wliu88/affordance_net/tree/50348e89339756a933e047a7fe4b94adfe35cd5b).
Using this wrapper, AffordanceNet will be running in the background as a ros service server. 

## Installation
- Install AffordanceNet as a library by following instructions in `/lib/affordance_net/README.md`. 
- Test the installation by running the demo on static images.
- No need to run the robot demo mentioned in that README since we can test it on the robot using the *better* wrapper here.

## Run Affordance Detection on Fetch (and Possibly Other Robots)
- Make sure you are receiving RGBD messages in ros. Check with `rostopic list`
- Modify `/launch/recognition.launch` if camera topics are different depending on your robot
- Compile this ros package: `catkin_make` or `catkin build`
- Start the server: `roslaunch rail_part_affordance_detection recognition.launch`
- Make the service call: `rosservice call /rail_part_affordance_detection/detect`

## Use The Service in Code
- Since affordance detection is now wrapped as a ros service, you can also call this service in code using ROS API for [Services](http://wiki.ros.org/Services)
- Check out `/srv` to see the service definition. 
