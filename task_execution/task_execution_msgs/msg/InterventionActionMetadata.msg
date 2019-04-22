# A message type to provide information about actions taken during an
# intervention

# The name of the action. Segmented by the broad classes

# Movement
string LOOK_UP=look_up
string LOOK_DOWN=look_down
string LOOK_LEFT=look_left
string LOOK_RIGHT=look_right

string MOVE_FORWARD=move_forward
string MOVE_BACKWARD=move_backward
string MOVE_LEFT=move_left
string MOVE_RIGHT=move_right
string MOVE_WAYPOINT=move_waypoint

string TORSO_UP=torso_up
string TORSO_DOWN=torso_down

string ARM_POSITION=arm_position
string ARM_LINEAR_UP=arm_linear_up
string ARM_LINEAR_DOWN=arm_linear_down
string ARM_LINEAR_LEFT=arm_linear_left
string ARM_LINEAR_RIGHT=arm_linear_right
string ARM_LINEAR_FORWARD=arm_linear_forward
string ARM_LINEAR_BACKWARD=arm_linear_backward
string ARM_ANGULAR_ROLL_LEFT=arm_angular_roll_left
string ARM_ANGULAR_ROLL_RIGHT=arm_angular_roll_right
string ARM_ANGULAR_PITCH_DOWN=arm_angular_pitch_down
string ARM_ANGULAR_PITCH_UP=arm_angular_pitch_up
string ARM_ANGULAR_YAW_LEFT=arm_angular_yaw_left
string ARM_ANGULAR_YAW_RIGHT=arm_angular_yaw_right

# Perception
string CROP_IMAGE=crop_image
string SEGMENT_IMAGE=segment_image

# Beliefs and Assumptions
string RELOCALIZE=relocalize
string UPDATE_BELIEFS=update_beliefs
# TODO: Kill and restart components, such as wifi, should also be actions

string type

# Args to the action (optional). This is a pickled byte array
string args
