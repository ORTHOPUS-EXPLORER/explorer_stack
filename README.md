## Demo use of ROS2 hardware interface for Explorer

1. Setup virtual can interface for simulator

```
modprobe vcan
ip link add dev vcan0 type vcan
ip link set mtu 16 up dev vcan0
```

2. Launch Simulator
```
ros2 run pyvesc_explorer app_sim
```

3. Launch Robot controller (and Explorer Comm bridge)
```
ros2 launch explorer_bringup hardware_base.launch.py use_bridge:=true
```

4. Publish some commands
```
ros2 launch explorer_bringup cartesian_control.launch.py
```

Notes:
- Setup should be quite similar for actual robot, just edit the config files.
- Only position interface is supported for now
- The bridge only supports position commands for now.
