#!/bin/bash

# Install Python dependencies for explorer_user_interfaces_web
# This script should be run after building the ROS2 package

echo "Installing Python dependencies for explorer_user_interfaces_web..."

# Get the path to the requirements.txt file
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REQUIREMENTS_FILE="$SCRIPT_DIR/../share/explorer_user_interfaces_web/requirements.txt"

if [ -f "$REQUIREMENTS_FILE" ]; then
    echo "Installing from $REQUIREMENTS_FILE"
    pip3 install -r "$REQUIREMENTS_FILE"
    echo "Python dependencies installed successfully!"
else
    echo "Requirements file not found. Installing dependencies manually..."
    pip3 install fastapi uvicorn[standard] websockets jinja2 PyYAML
fi

echo "Installation complete!"
