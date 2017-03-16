uint8 NUM_ACTUATOR_CONTROLS = 8
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 4
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 INDEX_FLAPS = 4
uint8 INDEX_SPOILERS = 5
uint8 INDEX_AIRBRAKES = 6
uint8 INDEX_LANDING_GEAR = 7
uint8 GROUP_INDEX_ATTITUDE = 0
uint8 GROUP_INDEX_ATTITUDE_ALTERNATE = 1
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[8] ghost_control

# TOPICS actuator_ghost_controls actuator_ghost_controls_0 actuator_ghost_controls_1 actuator_ghost_controls_2 actuator_ghost_controls_3
# TOPICS actuator_ghost_controls_virtual_fw actuator_ghost_controls_virtual_mc

