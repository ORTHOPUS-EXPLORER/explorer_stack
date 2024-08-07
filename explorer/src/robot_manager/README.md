# Robot manager file

Each file is independent

## inverse_kinematic

Calculates the robot's inverse kinematics.

* Uses moveit to obtain the robot's kinematic model from the urdf and srdf files.

* Uses the types declared in the "types" folder.

* Uses qpOASES

## forward_kinematic

Calculates the robot's forward kinematics

* Use moveit to obtain the robot's kinematic model from the urdf and srdf files.

* Uses the types declared in the "types" folder.

## velocity_integrator

Integrates joint velocities to obtain joint positions.

* Uses the types declared in the "types" folder.

# Usage

These are used by "spacenav_trajectory_qp": 

* forward_kinematic to get the cartesian position of the effector from the robot's joint positions

* inverse_kinematic to get the desired joint velocities from the desired cartesian effector velocities, the actual cartesian position of the effector and the robot's actual joint positions

* velocity_integrator to get the desired joint positions from the desired joint velocities.

