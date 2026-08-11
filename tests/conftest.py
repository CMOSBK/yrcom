"""Test fixtures — no robot required. The `robot` fixture provides a stateful
ES mock (TCP) that the real YaskawaClient talks to."""
from __future__ import annotations

import pytest

from mock_robot import MockRobotServer


@pytest.fixture(scope="session")
def robot_server():
    with MockRobotServer() as srv:
        yield srv


@pytest.fixture
def robot(robot_server):
    """Reset the mock state per test; returns the server itself."""
    robot_server.reset_state()
    return robot_server
