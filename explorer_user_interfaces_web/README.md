# Explorer User Interfaces Web

Web-based GUI for the Explorer robot control interfaces.

## Overview

This package provides a simplified real-time web interface for monitoring the Explorer robot's control mode and status. It displays:

- **Current control mode** (B1, B2, B3, A1, etc.) with large, clear text
- **Mode-specific image** showing the current control mode visually
- **Status LED** - a colored dot controlled by an integer topic with 8 different colors (0-7)

## Features

- **Real-time Updates**: WebSocket connection for live status updates
- **Minimal Design**: Clean, focused interface showing only essential information
- **LED Status Indicator**: Color-coded dot with 8 different states:
  - 0: Gray (Default/Unknown)
  - 1: Red 
  - 2: Orange/Amber (with blink animation)
  - 3: Green
  - 4: Blue (with pulse animation)
  - 5: Purple
  - 6: Pink
  - 7: Cyan
- **Mode Visualization**: Displays current mode and corresponding images
- **Responsive Design**: Works on desktop and mobile devices

## Dependencies

- ROS2 (Humble or later)
- Python 3.8+
- FastAPI
- Uvicorn
- WebSockets
- PyYAML
- Jinja2

## Installation

1. **Install ROS2 dependencies with rosdep:**
```bash
cd /path/to/your/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

2. **Install additional Python dependencies:**
```bash
# Navigate to the package directory
cd src/explorer_stack/explorer_user_interfaces_web

# Install Python dependencies that are not available through rosdep
pip3 install -r requirements.txt
```

3. **Build the package:**
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select explorer_user_interfaces_web
source install/setup.bash
```

## Usage

### Basic Launch

Launch the web GUI server:
```bash
ros2 launch explorer_user_interfaces_web web_gui.launch.py
```

### Custom Configuration

Launch with custom settings:
```bash
ros2 launch explorer_user_interfaces_web web_gui.launch.py port:=8080 host:=localhost mode_config_path:=/path/to/config.yaml
```

### Parameters

- `port` (default: 8080): Port for the web server
- `host` (default: 0.0.0.0): Host address for the web server
- `mode_config_path`: Path to the mode configuration YAML file

## Accessing the Web Interface

Once launched, open your web browser and navigate to:
```
http://localhost:8080
```

Or replace `localhost` with the appropriate IP address if accessing remotely.

## Integration with Explorer System

This package subscribes to the following ROS2 topics:

- `/command_node/mode_name` (std_msgs/String): Current control mode
- `/explorer_gui/led_color` (std_msgs/Int32): LED color value (0-7)

## Configuration

The web interface reads mode configuration from the same YAML files used by the `explorer_user_interfaces_cpp` package, typically located at:
```
explorer_user_interfaces_cpp/config/config_mode_0.yaml
```

## File Structure

```
explorer_user_interfaces_web/
├── explorer_user_interfaces_web/
│   ├── web_gui_node.py       # Main ROS2 node
│   ├── web_server.py         # FastAPI web server
│   ├── static/
│   │   ├── style.css         # CSS styling
│   │   └── app.js            # Frontend JavaScript
│   └── templates/
│       └── index.html        # Main web page
├── launch/
│   └── web_gui.launch.py     # Launch file
└── package.xml               # Package configuration
```

## Development

To modify the web interface:

1. Edit the HTML template in `templates/index.html`
2. Update styles in `static/style.css`
3. Modify frontend behavior in `static/app.js`
4. Adjust server logic in `web_server.py`

The server automatically serves static files, so changes to CSS/JS are immediately visible after refreshing the browser.

## Troubleshooting

### Web Server Won't Start
- Check that the port is not already in use
- Verify ROS2 dependencies are installed
- Check the console output for error messages

### WebSocket Connection Issues
- Ensure both the web server and the command_node are running
- Check firewall settings if accessing remotely
- Verify the correct WebSocket URL in browser developer tools

### Images Not Displaying
- Ensure image files are placed in `static/images/` directory
- Check that image filenames in YAML config match actual files
- Verify image file permissions
