# A message type to provide information about a task step

# The UUID of the step.
string uuid

# The actionlib_msgs/GoalStatus status of the step
uint8 status

# Any other step context information to be included in the trace.
# This is a pickled byte array
string context
