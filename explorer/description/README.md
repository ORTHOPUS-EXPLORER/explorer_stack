# URDF 

## explorer_POC1_description.urdf.xacro

This is the description of the "snake like" explorer. 

## explorer_new_description.urdf.xacro

This is the description of an other version of the explorer but it is currently not used and the masses and inertias are not right.

## explorer.urdf.xacro

This URDF combine explorer_old_description, gripper_pincette_description, explorer.ros2_control and explorer.gazebo. It is the URDF used for the simulation.

# ros2_control

## explorer.ros2_control.xacro

The `ros2_control` tag specifies hardware configuration of the robot and allows to access and control the robot interfaces. 

# gazebo

## explorer.gazebo.xacro

Gazebo plugin parses the `ros2_control` tags and loads the appropriate hardware interfaces and controller manager
