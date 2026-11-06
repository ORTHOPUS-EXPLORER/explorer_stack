"""Check that every page opened on the web GUI shows the same robot state."""

import asyncio
import json
import socket
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import uvicorn
import websockets
from std_msgs.msg import Bool, Int32, String

from explorer_user_interfaces_web.ros_bridge import CLIENT_QUEUE_SIZE, RosBridge
from explorer_user_interfaces_web.web_server import create_app

TIMEOUT_SECONDS = 5.0


def fake_node():
    node = MagicMock()
    node.has_parameter.return_value = False
    node.get_parameter.return_value.value = "/camera"
    return node


@pytest.fixture
def server():
    """Real web server on a free port. Tests act as the ROS thread by calling the callbacks."""
    node = fake_node()
    app = create_app(node)
    listening_socket = socket.socket()
    listening_socket.bind(("127.0.0.1", 0))
    uvicorn_server = uvicorn.Server(uvicorn.Config(app, log_level="critical"))
    thread = threading.Thread(
        target=uvicorn_server.run, kwargs={"sockets": [listening_socket]}, daemon=True
    )
    thread.start()

    deadline = time.monotonic() + TIMEOUT_SECONDS
    while not uvicorn_server.started:
        assert time.monotonic() < deadline, "Web server didn't start"
        time.sleep(0.01)

    port = listening_socket.getsockname()[1]
    yield SimpleNamespace(
        node=node, bridge=app.state.ros_bridge, url=f"ws://127.0.0.1:{port}/ws"
    )

    uvicorn_server.should_exit = True
    thread.join(TIMEOUT_SECONDS)


async def receive(page):
    return json.loads(await asyncio.wait_for(page.recv(), TIMEOUT_SECONDS))


async def receive_speed_levels(page, count):
    """Next \\p count speed levels sent to the page, the initial message being already received"""
    return [(await receive(page))["speed_level"] for _ in range(count)]


async def wait_for_client_count(bridge, count):
    deadline = time.monotonic() + TIMEOUT_SECONDS
    while len(bridge.connected_clients) != count:
        assert time.monotonic() < deadline, "Closed page wasn't removed"
        await asyncio.sleep(0.01)


# websocket_endpoint : a new page receives the drink, gripper lock and gripper direction state
def test_new_page_receives_shared_state(server):
    # Arrange
    server.bridge.mode_config = {
        "button_mappings": {"DRINK": {"axes": [{"control_name": "drink"}]}}
    }
    server.bridge.mode_callback(String(data="DRINK"))
    server.bridge.gripper_lock_callback(Bool(data=True))
    server.bridge.set_gripper_closure_direction("haut")

    async def scenario():
        async with websockets.connect(server.url) as page:
            return await receive(page)

    # Act
    initial = asyncio.run(scenario())

    # Assert
    assert initial["type"] == "initial"
    assert initial["mode"] == "DRINK"
    assert initial["drink_active"] is True
    assert initial["gripper_lock_active"] is True
    assert initial["gripper_closure_direction"] == "haut"


# websocket_endpoint : a new page receives the state of the joints after the initial message
def test_new_page_receives_joint_states(server):
    # Arrange
    server.bridge.joint_state_callback(
        1, SimpleNamespace(joint_name="joint_1", mode="position", state="Enable", error="")
    )

    async def scenario():
        async with websockets.connect(server.url) as page:
            await receive(page)
            return await receive(page)

    # Act
    joint_state = asyncio.run(scenario())

    # Assert
    assert joint_state["type"] == "joint_state_update"
    assert joint_state["state"] == "Enable"


# speed_level_callback : every update reaches every page, in order
def test_every_page_receives_every_update(server):
    # Arrange
    levels = list(range(1, 21))

    async def scenario():
        async with websockets.connect(server.url) as page_a:
            async with websockets.connect(server.url) as page_b:
                await receive(page_a)
                await receive(page_b)

                # Act
                for level in levels:
                    server.bridge.speed_level_callback(Int32(data=level))

                return await asyncio.gather(
                    receive_speed_levels(page_a, len(levels)),
                    receive_speed_levels(page_b, len(levels)),
                )

    levels_a, levels_b = asyncio.run(scenario())

    # Assert
    assert levels_a == levels
    assert levels_b == levels


# websocket_endpoint : a page closed is removed, and the others keep receiving updates
def test_closed_page_does_not_disturb_the_others(server):
    async def scenario():
        async with websockets.connect(server.url) as page_b:
            await receive(page_b)
            async with websockets.connect(server.url) as page_a:
                await receive(page_a)

            # Act
            await wait_for_client_count(server.bridge, 1)
            server.bridge.speed_level_callback(Int32(data=3))

            return await receive_speed_levels(page_b, 1)

    levels = asyncio.run(scenario())

    # Assert
    assert levels == [3]


# websocket_endpoint : a page closing normally isn't reported as an error
def test_closed_page_logs_no_error(server):
    async def scenario():
        async with websockets.connect(server.url) as page:
            await receive(page)

        # Act
        await wait_for_client_count(server.bridge, 0)

    asyncio.run(scenario())

    # Assert
    server.node.get_logger.return_value.error.assert_not_called()


# set_gripper_closure_direction : a direction selected on a page reaches the other pages
def test_gripper_direction_reaches_other_pages(server):
    async def scenario():
        async with websockets.connect(server.url) as page_a:
            async with websockets.connect(server.url) as page_b:
                await receive(page_a)
                await receive(page_b)

                # Act
                await page_a.send(
                    json.dumps({"type": "gripper_closure_direction", "direction": "gauche"})
                )

                return await receive(page_b)

    update = asyncio.run(scenario())

    # Assert
    assert update["type"] == "gripper_closure_direction_update"
    assert update["direction"] == "gauche"


# set_gripper_closure_direction : a direction selected on a page is published on ROS
def test_gripper_direction_is_published(server):
    async def scenario():
        async with websockets.connect(server.url) as page:
            await receive(page)

            # Act
            await page.send(
                json.dumps({"type": "gripper_closure_direction", "direction": "droite"})
            )

            await receive(page)

    asyncio.run(scenario())

    # Assert
    publisher = server.node.create_publisher.return_value
    assert publisher.publish.call_args.args[0].data == "droite"


# set_gripper_closure_direction : an invalid direction isn't sent to the pages
def test_invalid_gripper_direction_is_ignored(server):
    async def scenario():
        async with websockets.connect(server.url) as page:
            await receive(page)

            # Act
            for direction in ("sideways", "bas"):
                await page.send(
                    json.dumps({"type": "gripper_closure_direction", "direction": direction})
                )

            return await receive(page)

    update = asyncio.run(scenario())

    # Assert: "bas" is the first update received, "sideways" would come before it
    assert update["direction"] == "bas"


# _dispatch_update : a page too slow to read its updates still ends up on the current state
def test_slow_page_gets_current_state():
    # Arrange
    bridge = RosBridge(fake_node())
    last_level = 2 * CLIENT_QUEUE_SIZE

    async def scenario():
        bridge.set_event_loop(asyncio.get_running_loop())
        client_queue = bridge.add_client()

        # Act: never read while updates are published
        for level in range(1, last_level + 1):
            bridge.speed_level_callback(Int32(data=level))
            await asyncio.sleep(0)
        await asyncio.sleep(0)

        messages = []
        while not client_queue.empty():
            messages.append(client_queue.get_nowait())
        return messages

    messages = asyncio.run(scenario())

    # Assert: the backlog was replaced by the state when full, so nothing above the size remains
    assert len(messages) <= CLIENT_QUEUE_SIZE
    assert [m["speed_level"] for m in messages if "speed_level" in m][-1] == last_level


# _dispatch_update : a page too slow to read its updates doesn't make another page lose any
def test_slow_page_does_not_disturb_the_others():
    # Arrange
    bridge = RosBridge(fake_node())
    levels = list(range(1, 2 * CLIENT_QUEUE_SIZE + 1))

    async def scenario():
        bridge.set_event_loop(asyncio.get_running_loop())
        bridge.add_client()
        fast_queue = bridge.add_client()
        fast_queue.get_nowait()
        received = []

        # Act
        for level in levels:
            bridge.speed_level_callback(Int32(data=level))
            received.append((await asyncio.wait_for(fast_queue.get(), TIMEOUT_SECONDS)))

        return received

    received = asyncio.run(scenario())

    # Assert
    assert [message["speed_level"] for message in received] == levels
