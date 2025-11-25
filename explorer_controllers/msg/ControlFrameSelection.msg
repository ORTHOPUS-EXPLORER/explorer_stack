# Control frame selection message
# Allows runtime switching of control frames via topic instead of parameters

# Control frame options
uint8 FRAME_WORLD = 0
uint8 FRAME_TOOL = 1
uint8 FRAME_DRINK_SMALL = 2
uint8 FRAME_DRINK_BIG = 3

# Control frames to set
uint8 position_control_frame
uint8 orientation_control_frame
