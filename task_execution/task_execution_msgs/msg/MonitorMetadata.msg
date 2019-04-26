# A message type to provide information about a monitor event

# Is the monitor in a fault state or not
uint8 NOMINAL=0
uint8 FAULT=1
uint8 UNKNOWN=255
uint8 fault_status

# Context. This is a pickled byte array
string context

# The information is about the topics, services, actions, and/or nodes that
# might be affected
string[] topics
string[] services
string[] actions
string[] nodes
