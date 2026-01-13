#!/usr/bin/env python3

import os
import yaml
import asyncio
import json
import threading
import time
import queue
from typing import Dict, Optional
from pathlib import Path

from fastapi import FastAPI, WebSocket, Request, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Joy


class RosBridge:
    """Bridge between ROS2 and the web interface"""
    
    def __init__(self, ros_node: Node):
        self.node = ros_node
        self.current_mode = "Unknown"
        self.speed_level = None
        self.retract_status = "not ready"  # Retract status: ready, in progress, retracted, not ready
        self.connected_clients: set = set()
        
        # Use a thread-safe queue for updates
        self.update_queue = queue.Queue()
        
        # Load mode config once at startup
        config_path = ros_node.get_parameter('mode_config_path').value if ros_node.has_parameter('mode_config_path') else None
        self.mode_config = load_mode_config(config_path)
        
        # ROS2 subscribers
        self.mode_subscriber = self.node.create_subscription(
            String,
            '/command_node/mode_name',
            self.mode_callback,
            10
        )

        # Speed level subscriber - integer topic
        self.speed_level_subscriber = self.node.create_subscription(
            Int32,
            '/command_node/speed_level',
            self.speed_level_callback,
            10
        )

        # Retract status subscriber - string topic
        self.retract_status_subscriber = self.node.create_subscription(
            String,
            '/command_node/retract_status',
            self.retract_status_callback,
            10
        )

        self.node.get_logger().info('ROS Bridge initialized')
    
    def mode_callback(self, msg: String):
        """Callback for mode changes"""
        new_mode = msg.data
        # Only process if mode actually changed
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            self.node.get_logger().info(f'Mode changed to: {self.current_mode}')
            # Check drink axis in config
            drink_active = False
            # Assume self.mode_config is loaded and available
            button_mappings = getattr(self, 'mode_config', {}).get('button_mappings', {})
            mapping = button_mappings.get(new_mode, {})
            axes = mapping.get('axes', [])
            for axis in axes:
                if axis.get('control_name', '').lower() == 'drink':
                    drink_active = True
                    break
            print(f"[DEBUG] Backend: drink_active for mode {new_mode}: {drink_active}")
            # Put update in queue for drink LED
            self.update_queue.put({
                "type": "drink_led_update",
                "active": drink_active,
                "mode": self.current_mode,
                "timestamp": time.time()
            })
            # Put update in queue for mode
            self.update_queue.put({
                "type": "mode_update",
                "mode": self.current_mode,
                "timestamp": time.time()
            })

    def speed_level_callback(self, msg: Int32):
        """Callback for speed level changes"""
        new_level = msg.data
        if new_level != self.speed_level:
            self.speed_level = new_level
            self.node.get_logger().info(f'Speed level changed to: {self.speed_level}')
            self.update_queue.put({
                "type": "speed_level_update",
                "speed_level": self.speed_level,
                "timestamp": time.time()
            })

    def retract_status_callback(self, msg: String):
        """Callback for retract status changes"""
        new_status = msg.data
        if new_status != self.retract_status:
            self.retract_status = new_status
            self.node.get_logger().info(f'Retract status changed to: {self.retract_status}')
            self.update_queue.put({
                "type": "retract_status_update",
                "retract_status": self.retract_status,
                "timestamp": time.time()
            })
    
    async def get_updates(self):
        """Get any pending updates from the queue"""
        updates = []
        try:
            while not self.update_queue.empty():
                update = self.update_queue.get_nowait()
                updates.append(update)
        except queue.Empty:
            pass
        return updates
    
    async def broadcast_to_clients(self, message):
        """Broadcast message to all connected clients"""
        if not self.connected_clients:
            return
            
        disconnected = set()
        for websocket in self.connected_clients:
            try:
                await websocket.send_text(json.dumps(message))
            except Exception as e:
                self.node.get_logger().debug(f'Failed to send to client: {e}')
                disconnected.add(websocket)
        
        # Clean up disconnected clients
        self.connected_clients -= disconnected
    
    async def broadcast_mode_update(self):
        """Send mode update to all connected clients"""
        print(f"DEBUG: broadcast_mode_update called, clients: {len(self.connected_clients)}")
        
        if not self.connected_clients:
            print("DEBUG: No connected clients")
            return
            
        message = {
            "type": "mode_update",
            "mode": self.current_mode,
            "timestamp": asyncio.get_event_loop().time()
        }
        
        print(f"DEBUG: Sending message: {message}")
        
        disconnected = set()
        for websocket in self.connected_clients:
            try:
                await websocket.send_text(json.dumps(message))
                print("DEBUG: Message sent successfully to client")
            except Exception as e:
                print(f"DEBUG: Error sending message: {e}")
                disconnected.add(websocket)
        
        # Clean up disconnected clients
        self.connected_clients -= disconnected
    
    async def broadcast_status_update(self):
        """Send status update to all connected clients"""
        if not self.connected_clients:
            return
            
        message = {
            "type": "status_update",
            "status": self.connection_status,
            "timestamp": asyncio.get_event_loop().time()
        }
        
        disconnected = set()
        for websocket in self.connected_clients:
            try:
                await websocket.send_text(json.dumps(message))
            except:
                disconnected.add(websocket)
        
        self.connected_clients -= disconnected


def load_mode_config(config_path: str) -> Dict:
    """Load mode configuration from YAML file"""
    if not config_path or not os.path.exists(config_path):
        return {
            "mode_info": {
                "name": "Unknown",
                "display_name": "No Configuration",
                "description": "Configuration file not found",
                "default_image": "default.png"
            },
            "button_mappings": {}
        }
    
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config: {e}")
        return {"mode_info": {}, "button_mappings": {}}


def create_app(ros_node: Node) -> FastAPI:
    """Create and configure the FastAPI application"""
    
    app = FastAPI(title="Explorer Robot Web GUI")
    
    # Get the package directory for static files
    package_dir = Path(__file__).parent
    
    # Mount static files
    app.mount("/static", StaticFiles(directory=str(package_dir / "static")), name="static")
    
    # Templates
    templates = Jinja2Templates(directory=str(package_dir / "templates"))
    
    # Create ROS bridge
    ros_bridge = RosBridge(ros_node)
    
    # Store references for access in routes
    app.state.ros_bridge = ros_bridge
    app.state.ros_node = ros_node
    app.state.templates = templates
    
    @app.get("/", response_class=HTMLResponse)
    async def home(request: Request):
        """Main page"""
        # Load mode configuration
        config_path = ros_node.get_parameter('mode_config_path').value
        mode_config = load_mode_config(config_path)
        
        context = {
            "request": request,
            "current_mode": ros_bridge.current_mode,
            "mode_config": mode_config
        }
        return templates.TemplateResponse("index.html", context)
    
    @app.get("/api/status")
    async def get_status():
        """Get current status"""
        return {
            "mode": ros_bridge.current_mode,
            "retract_status": ros_bridge.retract_status,
            "speed_level": ros_bridge.speed_level,
            "timestamp": time.time()
        }
    
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        """WebSocket endpoint for real-time updates"""
        await websocket.accept()
        ros_bridge.connected_clients.add(websocket)
        
        try:
            # Send initial status
            initial_message = {
                "type": "initial",
                "mode": ros_bridge.current_mode,
                "speed_level": ros_bridge.speed_level,
                "retract_status": ros_bridge.retract_status
            }
            await websocket.send_text(json.dumps(initial_message))
            
            # Keep connection alive and check for updates
            while True:
                try:
                    # Check for updates from ROS
                    updates = await ros_bridge.get_updates()
                    
                    # Send any pending updates
                    for update in updates:
                        await websocket.send_text(json.dumps(update))
                    
                    # Wait a bit or for a client message
                    try:
                        await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
                    except asyncio.TimeoutError:
                        # No message from client, continue checking for updates
                        pass
                        
                except Exception as e:
                    ros_node.get_logger().error(f"WebSocket error: {e}")
                    break
                    
        except WebSocketDisconnect:
            ros_bridge.connected_clients.discard(websocket)
        except Exception as e:
            ros_node.get_logger().error(f"WebSocket error: {e}")
            ros_bridge.connected_clients.discard(websocket)
    
    return app
