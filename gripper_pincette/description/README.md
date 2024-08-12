# URDF 

## gripper_pincette_description.urdf.xacro

This is the description of the gripper pincette. 

## gripper_pincette.urdf.xacro

This URDF combine gripper_pincette_description, gripper_pincette.ros2_control and gripper_pincette.gazebo. It is the URDF used for the simulation.

# ros2_control

## gripper_pincette.ros2_control.xacro

The `ros2_control` tag specifies hardware configuration of the robot and allows to access and control the robot interfaces. 

# gazebo

## gripper_pincette.gazebo.xacro

Gazebo plugin parses the `ros2_control` tags and loads the appropriate hardware interfaces and controller manager

