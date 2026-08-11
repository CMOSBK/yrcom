"""yrcom — Yaskawa robot communication (Ethernet Server + HSES + File Control).

Pure-stdlib library. Three transports under one roof:

- `YaskawaClient`   — classic Ethernet Server (TCP 80, host-control commands)
- `HsesClient`      — High Speed Ethernet Server (UDP 10040, 45 commands)
- `HsesFileClient`  — HSES File Control (UDP 10041, read/write job files)
- `RobotConnection` — bundles ES + HSES behind one IP, selectable via `transport`
- `HsesReplayServer`— replays a recorded session as a UDP server (offline dev/testing)
"""
from .yaskawa_client import VarType, YaskawaClient, EthernetServerClient
from .hses_client import (
    HsesClient,
    HsesError,
    PositionVar,
    AxisPositionVar,
    CartesianMove,
    PulseMove,
    MODEL_DX100,
    MODEL_DX200,
    MODEL_YRC1000,
)
from .hses_errors import status_text, added_status_text
from .hses_file import HsesFileClient, HSES_FILE_PORT
from .robot_connection import (
    RobotConnection,
    PersistentTcpClient,
    HsesWriteBlockedError,
    TRANSPORT_HSES,
    TRANSPORT_ETHERNET,
    HSES_PORT,
    TCP_PORT,
)
from .hses_sim import HsesReplayServer

__version__ = "0.2.1"

__all__ = [
    "YaskawaClient", "VarType", "EthernetServerClient",
    "HsesClient", "HsesError",
    "PositionVar", "AxisPositionVar", "CartesianMove", "PulseMove",
    "MODEL_DX100", "MODEL_DX200", "MODEL_YRC1000",
    "status_text", "added_status_text",
    "HsesFileClient", "HSES_FILE_PORT",
    "RobotConnection", "PersistentTcpClient", "HsesWriteBlockedError",
    "TRANSPORT_HSES", "TRANSPORT_ETHERNET", "HSES_PORT", "TCP_PORT",
    "HsesReplayServer",
]
