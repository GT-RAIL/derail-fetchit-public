# A message type to provide an execution trace

# A header indicating the time of the trace update
time stamp

# A human readable name of the event
string name

# The type of the event
uint8 TASK_STEP_EVENT=1
uint8 ROSGRAPH_EVENT=2
uint8 MONITOR_EVENT=3
uint8 BELIEF_EVENT=4
uint8 type

# The different types of events sometimes have associated metadata
TaskStepMetadata task_step_metadata
MonitorMetadata monitor_metadata
BeliefMetadata belief_metadata
