# ROS2_control_explorer

### Install ROS2 Control packages

```
sudo apt install ros-humble-ros2-control 
sudo apt install ros-humble-ros2-controllers
```

### Install connector between gazebo and ros2_control

```
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros2-control-demos
```

### Install Spacenav driver (optionnal, to control robot using a SpaceNav from 3Dconnexion)

```
sudo apt install spacenavd
sudo apt install libspnav-dev
sudo apt install ros-humble-spacenav
```

In the gazebo configuration file ~/.gazebo/gui.ini, add the following lines (otherwise, spacenav will move the gazebo camera):
```
[spacenav]
deadband_x = 0.1
deadband_y = 0.1
deadband_z = 0.1
deadband_rx = 0.1
deadband_ry = 0.1
deadband_rz = 0.1
topic=~/spacenav/remapped_joy_topic_to_something_not_used
```

# Run

## Run explorer alone

To check that the robot descriptions are working properly use following launch commands

```
ros2 launch ros2_control_explorer view_explorer.launch.py
```

The joint_state_publisher_gui provides a GUI to change the configuration for RRbot. It is immediately displayed in RViz.

To start the robot with the hardware interface instead of the simulators, open a terminal, source your ROS2-workspace and execute its launch file with

```
ros2 launch ros2_control_explorer explorer.launch.py
```

To start the robot in the simulators, open a terminal, source your ROS2-workspace first. Then, execute the launch file with
```
ros2 launch ros2_control_explorer explorer_gazebo_classic.launch.py gui:=true
```

Then, open a new terminal and run the following command.

```
ros2 launch ros2_control_explorer send_trajectory.launch.py
```

You should see the robot making a circular motion in RViz and Gazebo.


You can move the robot with cartesian control using a 3D mouse or the GUI by launching this command.

* With KDL Solver
```
ros2 launch ros2_control_explorer explorer_spacenav.launch.py gui:=true
```

* With TRAC-IK Solver
```
ros2 launch ros2_control_explorer explorer_spacenav_ik.launch.py gui:=true
```

* With a QP
```
ros2 launch ros2_control_explorer explorer_spacenav_qp.launch.py gui:=true
```

Start the MoveIt demo to interactively plan and execute motions for the robot in RViz.
```
ros2 launch explorer_moveit_config demo.launch.py
```

## Run wheelchair alone

To check that the wheelchair descriptions are working properly use following launch commands

```
ros2 launch ros2_control_wheelchair display.launch.py

```

To start the wheelchair in the simulators, open a terminal, source your ROS2-workspace first. Then, execute the launch file with

```
ros2 launch ros2_control_wheelchair wheelchair_gazebo_classic.launch.py gui:=true
```

Then, open a new terminal and run the following command to move the wheelchair.

```
ros2 run ros2_control_wheelchair teleop
```

## Run wheelchair and explorer

To check that the descriptions are working properly use following launch commands

```
ros2 launch disabled_simulator display.launch.py
```

To start the wheelchair in the simulators, open a terminal, source your ROS2-workspace first. Then, execute the launch file with (not working)

```
ros2 launch disabled_simulator simulation.launch.py gui:=true
```
