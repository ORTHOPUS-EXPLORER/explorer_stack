## ROS2 hardware interface for Explorer : Launch mode_0 

## Install

`explorer_stack` targets ROS 2 **Jazzy** and depends on a git submodule (`libs/orthopus_vesc`), so make sure submodules are checked out / up to date:
```
git submodule update --init --recursive
```

Then choose one of the proposed options below.

### Devcontainer (recommended)

This repository ships a ready-to-use [Dev Container](.devcontainer/devcontainer.json) for VS Code (or any [Dev Containers](https://containers.dev/)-compatible editor).

1. Install [Docker](https://docs.docker.com/engine/install/) (ubuntu: expect issues installing with snap, prefer install through apt) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
2. Open the workspace folder in VS Code and select **Reopen in Container**.
3. The container builds the workspace automatically on creation (`.devcontainer/build.sh`). Once it's up, jump to next section to launch the stack.

**Tips: package joy for joystick input management doesn't deals with hot plug in Docker container, use ros parameter 'joy_backend:=joy_linux' to circumvent this issue**

### Ready-to-use Docker image

Prebuilt image are hosted on Github Registry and available publicly here: `ghcr.io/orthopus-explorer/explorer_stack/dev:jazzy-latest`, it includes development packages (build tools, debugging) and ros2 dependencies defined in this project.

Pull & run:
```
docker pull ghcr.io/orthopus-explorer/explorer_stack/dev:jazzy-latest
docker run -it --network=host --privileged --ipc=host --ulimit rtprio=99 --ulimit memlock=-1  --device /dev/dri:/dev/dri  --device /dev/input:/dev/input -e DISPLAY="$DISPLAY" --volume=/tmp/.X11-unix:/tmp/.X11-unix ghcr.io/orthopus-explorer/explorer_stack/dev:jazzy-latest bash
```

Or build an image locally instead of pulling:
```
docker build --target explorer_dev -t explorer_stack:dev .
```

### Plain ROS 2 install (no Docker)

1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) on your machine.
2. Clone this repo (with submodules) into a ros2 colcon workspace, for example `~/ros2_ws/src/explorer_stack`.
3. Install dependencies and build:
```
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Once installed, continue with the CAN interface setup and launch commands below.

## Launch ros2 stack

### Robot

1. Setup hardware can interface 

```
modprobe vcan
ip link set can0 up type can bitrate 1000000
ip link set can0 txqueuelen 100
```

2. Launch with default parameter
```
ros2 launch explorer_user_interfaces_cpp mode_0.launch.py
```


### Simulation 

1. Setup virtual can interface for simulation

```
modprobe vcan
ip link add dev vcan0 type vcan
ip link set mtu 16 up dev vcan0
```

2. Launch with can_port parameter set
```
ros2 launch explorer_user_interfaces_cpp mode_0.launch.py can_port:=vcan0
```

### Launch files

Several other launch files are worth mentioning / using (format: package  launch_file):
- GUI with cartesian control: explorer_bringup cartesian_control.launch.py
- GUI with joints control: explorer_bringup joint_control.launch.py
- Mode 0 but with impedance control controller enabled: explorer_user_interfaces_cpp mode_0_impedance.launch.py

## Impedance control

This section describes impedance control capabilities and usage

### CAUTION

Impedance control can be dangerous and should be used carefully. Setting wrong stiffness / damping parameters can cause the robot to fall or oscillate.

The parameter "safety_imp_max_q_error" represents basically a tracking error for impedance mode (max difference between setpoint and measured position). It needs to be large enough so the robot can deflect under external forces (as expected in impedance control mode) without triggering an error. However, keeping it as low as possible given the chosen stiffness / damping parameter is  required to ensure safety.

In particular when powered by AC adapter, playing too hard with the robot in impedance mode can reiject current into the AC adapter, causing it to fail and drop power, causing one or more joints to reboot and the robot to fall. Make sure to test your setup with caution (safe environment, emergency stop in hand) and test a bit harder than nominal condition before starting working with impedance mode.

### Overview

![control scheme](scripts/run/impedance/Impedance_scheme.jpg)

This control scheme describes the impedance control scheme:

- setpoints (pos, vel (not used yet) and torque) are streamed over CAN
- Stiffness and daping gains are used to compute a target torque that simulates a physical spring and damper between the target angle and the actual angle -> this computes a target torque 
- A PID torque controller closed the loop with torque sensor and outputs current setpoint, ensuring the target torque is followed. This PID can be bypassed to perform torque control based on motor current.
- The current setpoint is computed given the handled by low-level current loop (FOC) to control motor phases mosfets.
- State from the robot is fed back to each controller
- If limits are enabled, limits reaction have an impact only near the end limits and it simulates a physical end stop, preventing the robot to cross the joints limits. it computes an extra component to torque setpoint and can be tuned. 

### Basic usage

1. Bringup the robot in a mode that uses custom_controller, such as:
```
ros2 launch explorer_bringup custom_controller_joint_control.launch.py
```
or
```
ros2 launch explorer_user_interfaces_cpp mode_0_impedance.launch.py
```

2. Deploy the robot (for mode_0 example) in position mode (default mode)

3. Once the robot is deployed and clear from obstacles, you can switch one or more joints to impedance mode:
All joints one by one from 6 to one
```
./scripts/run/impedance/imp_all.sh
```
or
```
./scripts/run/impedance/impN.sh
```

4. You can try the current based control (uses torque estimation and control from motor current, switching off torque sensor)
```
./scripts/run/impedance/bypass_torque_sensor.sh
```

5. You can switch back to position mode when needed

**CAUTION**: the joints might jump back to desired position very fast if current position is far from desired position.

```
./scripts/run/impedance/pos_all.sh
```

### Live update of stiffness / damping

The stiffness and damping parameters are set for each joint at bringup, based on explorer_bringup/config/explorer_custom_controller.yaml configuration

It is possible to live stream stiffness / damping parameters to adapt the impedance behavior from ROS - making it possible to control a cartesian stiffness / damping by updating joint stiffness / damping on the current robot configuration.

Each joint has its own config topic such as for joint 1:
```
name: /explorer_joint_1/config type: orthopus_vesc_interfaces/msg/Config
```

the parameters are sent over CAN to the actuators at 10Hz


### Gravity compensation / torque feed forward

In impedance control mode, effort interface can be used to send torque commands (can be used to peerform gravity compensation). The topic is also exposed

```
ros2 topic pub /explorer_custom_controller/effort/commands std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0,0.0,0.0]}"
```

## Development / Code style

This project is using clang as [C++ linter](https://clang.llvm.org/extra/clang-tidy/) for code static analysis / good practice enforcer and as [C++ code formatter](https://clang.llvm.org/docs/ClangFormat.html), you could have a look here at their configuration here : [linting](.clang-format) and [formatter](.clang-tidy).

For Python code, we're using [ruff](https://github.com/astral-sh/ruff) as linter and formatter, check [ruff.toml](ruff.toml) file for its configuration.