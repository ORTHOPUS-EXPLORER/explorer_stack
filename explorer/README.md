# Explorer

This package allows to simulate the explorer using `ros2_control` and Gazebo

## View the URDF

To simply view the URDF in RViz, open a terminal and launch the `view_explorer.launch.py`.  

```
ros2 launch ros2_control_explorer view_explorer.launch.py
```

With the `joint_state_publisher_gui` you can now change the position of every joint.

## Launch the simulation

### Cartesian control with moveit and qpOASES using command position only

To launch the simulation in Gazebo, open a terminal and launch the `complete_system.launch.py`.  

```
ros2 launch ros2_control_explorer complete_system.launch.py
```

You can run the simulation with some arguments :

* `gui:=false` to deactivate RViz
* `spacenav:=false` to deactivate spacenav

To control the explorer you can use the GUI or a space mouse.

### Cartesian control with moveit and qpOASES using VESC simulation

First launch 

```
ros2 run pyvesc_explorer app_sim
```

Then open a terminal and launch the `explorer_real.launch.py`.  

```
ros2 launch ros2_control_explorer explorer_real.launch.py use_bridge:=true
```

You can run the simulation with some arguments :

* `gui:=true` to activate RViz
* `spacenav:=false` to deactivate spacenav

To control the explorer you can use the GUI or a space mouse.


