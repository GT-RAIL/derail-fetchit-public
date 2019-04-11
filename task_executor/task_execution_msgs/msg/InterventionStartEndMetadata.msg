# A message type to provide information about the start or end of the
# intervention event

# Status of the intervention
uint8 START=1
uint8 END=2
uint8 status

# The intervention request and response
RequestAssistanceGoal request
RequestAssistanceResult response
