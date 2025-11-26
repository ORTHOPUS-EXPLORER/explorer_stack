# Switch position frame to Tool, orientation frame to DrinkSmall
ros2 topic pub --once /explorer_controllers/qp_solving/control_frame_selection explorer_msgs/msg/ControlFrameSelection "{position_control_frame: 1, orientation_control_frame: 2}"

# Switch both frames to World
ros2 topic pub --once /explorer_controllers/qp_solving/control_frame_selection explorer_msgs/msg/ControlFrameSelection "{position_control_frame: 0, orientation_control_frame: 0}"

# Set position control frame to Tool (1), orientation to World (0)
ros2 param set /qp_solving position_control_frame Tool
ros2 param set /qp_solving orientation_control_frame World


# Available frames (topic):
0 = FRAME_WORLD
1 = FRAME_TOOL
2 = FRAME_DRINK_SMALL
3 = FRAME_DRINK_BIG

# Available frames (param):
World
Tool
DrinkSmall
DrinkBig