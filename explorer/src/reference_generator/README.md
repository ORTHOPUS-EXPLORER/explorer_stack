# spacenav_trajectory

* Use `kdl_parser` to extract the kinematic chain from the URDF

* Subscribe to a topic to get cartesian velocities from the space mouse or the GUI using

* Use `KDL::ChainIkSolverVel_pinv` to get joint positions and velocities from the desired cartesian velocities

* Publish the result in a topic for the explorer_controller (custom controller)

# spacenav_trajectory_qp

* Subscribe to a topic to get cartesian velocities and the gripper state from the space mouse or the GUI

* Use `forward_kinematic` to get the cartesian position of the effector from the robot's joint positions

* Use `inverse_kinematic` to get the desired joint velocities from the desired cartesian effector velocities, the actual cartesian position of the effector and the robot's actual joint positions

* Use `velocity_integrator` to get the desired joint positions from the desired joint velocities.

* Publish the result in a topic for the explorer_controller (custom controller)

* Publish the gripper state in a topic for the gripper_controller (`forward_command_controller`)