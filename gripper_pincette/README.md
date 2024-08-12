# Gripper pincette

This package allows to simulate a gripper using `ros2_control` and Gazebo

## View the URDF

To simply view the URDF in RViz, open a terminal and launch the `view_gripper.launch.py`.  

```
ros2 launch gripper_pincette view_gripper.launch.py
```

With the `joint_state_publisher_gui` you can now change the position of every joint.


## Launch the simulation

To launch the simulation in Gazebo, open a terminal and launch the `gripper_pincette.launch.py`.  

```
ros2 launch gripper_pincette gripper_pincette.launch.py
```

You can run the simulation with some arguments :

* `gui:=true` to activate RViz

To control the gripper, open a new terminal (not working for now).

* To close the gripper send  :

```
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1.05"

```

* To open the gripper send  :

```
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0"

```
