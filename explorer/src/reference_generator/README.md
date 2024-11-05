# input_integrator

* Ask for initial cartesian positions of the end effector

* Subscribe to topics to get the desired cartesian velocities

* Integrate the cartesian velocities to get cartesian positions

* Publish the desired cartesian positions

# qp_solving

* Use "forward_kinematic to get the initial cartesian positions of the end effector and send it to "input_integrator"

* Subscribe to topics to get the desired cartesian positions and velocities, actual joints positions and previous desired joints position

* Use "inverse_kinematic" to get desired joints velocities 

* Publish desired joints velocities

# output_integrator

* Subscribe to a topic to get desired joints velocities

* Integrate joints velocities to get joints positions

* Publish desired joints positions 

# test_output_integrator

* Subscribe to the cartesian velocities topic and publish it as joints velocities