#!/usr/bin/env python3

import asyncio
import json
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from rclpy.node import Node

from explorer_user_interfaces_web.ros_bridge import RosBridge

# How long the camera endpoint blocks waiting for a new frame before it timeouts and check for disconnection.
CAMERA_FRAME_WAIT_TIMEOUT = 4.0


def create_app(ros_node: Node) -> FastAPI:
    """Create and configure the FastAPI application"""

    app = FastAPI(title="Explorer Robot Web GUI")

    # Get the package share directory for static files
    package_name = "explorer_user_interfaces_web"
    try:
        share_dir = Path(get_package_share_directory(package_name))
        static_dir = share_dir / "static"
        templates_dir = share_dir / "templates"
    except Exception as e:
        # Fallback to source directory when running in development (not installed)
        ros_node.get_logger().warning(
            f"Could not resolve share directory for {package_name}: {e}"
        )
        package_dir = Path(__file__).parent
        static_dir = package_dir / "static"
        templates_dir = package_dir / "templates"
    # Mount static files

    app.mount(
        "/static",
        StaticFiles(directory=str(static_dir), follow_symlink=True),
        name="static",
    )

    # Templates
    templates = Jinja2Templates(directory=str(templates_dir))

    # Create ROS bridge
    ros_bridge = RosBridge(ros_node)

    # Store references for access in routes
    app.state.ros_bridge = ros_bridge
    app.state.ros_node = ros_node
    app.state.templates = templates

    @app.get("/", response_class=HTMLResponse)
    async def home(request: Request):
        """Main page"""
        context = {
            "current_mode": ros_bridge.current_mode,
            "mode_config": ros_bridge.mode_config,
        }
        return templates.TemplateResponse(
            request=request, name="index.html", context=context
        )

    @app.get("/api/status")
    async def get_status():
        """Get current status"""
        return {
            "mode": ros_bridge.current_mode,
            "retract_status": ros_bridge.retract_status,
            "speed_level": ros_bridge.speed_level,
            "timestamp": time.time(),
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
                "retract_status": ros_bridge.retract_status,
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

    @app.websocket("/ws/camera")
    async def websocket_camera_endpoint(websocket: WebSocket):
        """WebSocket endpoint for camera frames"""
        await websocket.accept()
        last_frame_index = 0

        try:
            while True:
                try:
                    frame, last_frame_index = await ros_bridge.get_latest_camera_frame(
                        last_frame_index, timeout=CAMERA_FRAME_WAIT_TIMEOUT
                    )
                    if frame is not None:
                        await websocket.send_bytes(frame)
                    else:
                        # Wait timed out, briefly check for client disconnect before waiting again.
                        try:
                            await asyncio.wait_for(
                                websocket.receive_text(), timeout=0.01
                            )
                        except asyncio.TimeoutError:
                            pass

                except Exception as e:
                    ros_node.get_logger().error(f"Camera WebSocket error: {e}")
                    break

        except WebSocketDisconnect:
            pass
        except Exception as e:
            ros_node.get_logger().error(f"Camera WebSocket error: {e}")

    return app
