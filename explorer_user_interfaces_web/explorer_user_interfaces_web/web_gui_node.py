#!/usr/bin/env python3

import asyncio
import threading

import rclpy
import uvicorn
from rclpy.node import Node

from .web_server import create_app


class WebGUINode(Node):
    """ROS2 Node that manages the web GUI server"""
    
    def __init__(self):
        super().__init__('web_gui_node')
        
        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('mode_config_path', '')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.mode_config_path = self.get_parameter('mode_config_path').value
        
        self.get_logger().info(f'Starting web GUI on {self.host}:{self.port}')
        
        # Create FastAPI app with ROS node reference
        self.app = create_app(self)
        
        # Start web server in a separate thread
        self.server_thread = threading.Thread(
            target=self._run_server, 
            daemon=True
        )
        self.server_thread.start()
        
        self.get_logger().info('Web GUI node initialized')
    
    def _run_server(self):
        """Run the FastAPI server in its own event loop"""
        try:
            # Run the server
            uvicorn.run(
                self.app, 
                host=self.host, 
                port=self.port,
                log_level="info",
                loop="asyncio"
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = WebGUINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
