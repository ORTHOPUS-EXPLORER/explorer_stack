# Command Node

`command_node.cpp` is a ROS2 node designed to read a YAML file describing the behavior of a control mode and manage it accordingly. It interprets the configuration and executes commands based on the specified mode.

## Launching the Node

To run the node, use the following command:

```
ros2 launch explorer_user_interfaces_cpp command_node.launch.py
```

## Visualization

To visualize the outputs of the node in real-time, you need to run PlotJuggler in parallel. This allows you to monitor and analyze the data produced by the node.
