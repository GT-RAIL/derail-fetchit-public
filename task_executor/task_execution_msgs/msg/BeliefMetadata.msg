# A message type to provide information about a belief event

# Depending on the belief updater, the "name" is one of a series of constants.
# Eg: object_location, torso_raised, gripper, etc. Consider these predicates
# Look at BeliefKeys.msg for the constant definition.

# A float value in [0, 1] for the value of the belief. Usually this will be 0 or
# 1, but we want the flexibility to specify probabilities in the future should
# the need arise
float32 value

# Context. This is a pickled byte array
string context
