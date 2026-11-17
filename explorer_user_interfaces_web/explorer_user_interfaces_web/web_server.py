#!/usr/bin/env python3

import asyncio
import json
import time
from contextlib import asynccontextmanager
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


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Give the server event loop to the ROS bridge, before any page can connect"""
    app.state.ros_bridge.set_event_loop(asyncio.get_running_loop())
    yield


def create_app(ros_node: Node) -> FastAPI:
    """Create and configure the FastAPI application"""

    app = FastAPI(title="Explorer Robot Web GUI", lifespan=lifespan)

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

    async def send_updates(websocket: WebSocket, client_queue: asyncio.Queue):
        """Send the updates of a page, in its own task so it can't delay the other pages"""
        while True:
            update = await client_queue.get()
            await websocket.send_text(json.dumps(update))

    async def receive_messages(websocket: WebSocket):
        """Handle the messages sent by a page"""
        while True:
            message_text = await websocket.receive_text()

            try:
                message = json.loads(message_text)
            except json.JSONDecodeError:
                ros_node.get_logger().warning("Invalid JSON received from WebSocket")
                continue

            # Handle gripper direction selection
            if isinstance(message, dict) and message.get("type") == "gripper_closure_direction":
                direction = message.get("direction")

                if isinstance(direction, str):
                    ros_bridge.set_gripper_closure_direction(direction)
                else:
                    ros_node.get_logger().warning("Invalid gripper direction message")

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        """WebSocket endpoint for real-time updates"""
        await websocket.accept()
        client_queue = ros_bridge.add_client()

        try:
            # The page is gone as soon as one of them stops: sending or receiving failed
            tasks = {
                asyncio.create_task(send_updates(websocket, client_queue)),
                asyncio.create_task(receive_messages(websocket)),
            }
            _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()

            for result in await asyncio.gather(*tasks, return_exceptions=True):
                if isinstance(result, Exception) and not isinstance(result, WebSocketDisconnect):
                    ros_node.get_logger().error(f"WebSocket error: {result}")
        finally:
            ros_bridge.remove_client(client_queue)

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
