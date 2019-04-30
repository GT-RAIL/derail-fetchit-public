# The different beliefs that can be updated by any node
# Ensure that the value of the message is a lowercase
# version of the key. Some of our task code assumes this
# to be the case

#### TASK state beliefs ####

# State of the large gear
string LARGE_GEAR_ON_TABLE=large_gear_on_table
string LARGE_GEAR_IN_SCHUNK=large_gear_in_schunk
string LARGE_GEAR_IN_KIT=large_gear_in_kit

# State of the small gear
string SMALL_GEAR_ON_TABLE=small_gear_on_table
string SMALL_GEAR_IN_KIT=small_gear_in_kit

# State of the bolts
string ZERO_BOLTS_IN_KIT=zero_bolts_in_kit
string ONE_BOLT_IN_KIT=one_bolt_in_kit
string TWO_BOLTS_IN_KIT=two_bolts_in_kit

# State of the kit
string KIT_ON_TABLE=kit_on_table
string KIT_ON_ROBOT=kit_on_robot
string KIT_COMPLETE=kit_complete

# State of the SCHUNK
string SCHUNK_IS_MACHINING=schunk_is_machining

#### ROBOT state beliefs ####

# Location of the robot
string ROBOT_AT_NAVIGATION_GOAL=robot_at_navigation_goal
string ROBOT_AT_KIT_STATION=robot_at_kit_station
string ROBOT_AT_SCHUNK_STATION=robot_at_schunk_station
string ROBOT_AT_GEARBOX_STATION=robot_at_gearbox_station
string ROBOT_AT_BOLT_STATION=robot_at_bolt_station
string ROBOT_AT_DROPOFF=robot_at_dropoff
