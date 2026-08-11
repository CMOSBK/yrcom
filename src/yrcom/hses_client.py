"""
Yaskawa YRC1000 — High-Speed Ethernet Server (HSES) client.
UDP port 10040, YERC binary protocol. No external dependencies.
"""
from __future__ import annotations

import logging
import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Optional, Self

from .hses_errors import status_text, added_status_text

# Diagnostic log of the HSES layer (requests/responses, control commands, errors).
# This is a *software* trace for field debugging — not the telemetry CSV. The
# library only emits records; the application decides handlers/levels. Requests
# and responses go to DEBUG, submitted control commands to INFO, and any robot
# error / malformed reply to ERROR.
log = logging.getLogger(__name__)

HSES_PORT = 10040
_MAGIC    = b"YERC"
_HDR_SIZE = 0x0020   # 32 bytes, always
_PROC_ROBOT = 1      # processing division: robot control

# Command numbers per HW1483358 Section 3.3.3.x
_CMD_ALARM    = 0x70  # Alarm Data Reading
_CMD_ALARM_HIST = 0x71  # Alarm History Reading
_CMD_STATUS   = 0x72  # Status Information Reading
_CMD_JOB      = 0x73  # Executing Job Information Reading
_CMD_AXIS_CFG = 0x74  # Axis Configuration Information Reading
_CMD_POSITION = 0x75  # Robot Position Data Reading
_CMD_POSERR   = 0x76  # Position Error (following error) Reading
_CMD_TORQUE   = 0x77  # Torque Data Reading
_CMD_IO       = 0x78  # I/O Data Reading
_CMD_REGISTER = 0x79  # Register Data Reading / Writing
_CMD_B_VAR    = 0x7A  # Byte Variable (B) Reading
_CMD_I_VAR    = 0x7B  # Integer Variable (I) Reading
_CMD_D_VAR    = 0x7C  # Double Variable (D) Reading / Writing
_CMD_R_VAR    = 0x7D  # Real Variable (R) Reading / Writing
_CMD_S_VAR    = 0x8C  # 32-Byte Character Variable (S) Reading / Writing (DX200/YRC1000)
_CMD_S16_VAR  = 0x7E  # 16-Byte Character Variable (S) Reading / Writing (DX100)
_CMD_P_VAR    = 0x7F  # Robot Position Variable (P) Reading / Writing
_CMD_BP_VAR   = 0x80  # Base Position Variable (BP) Reading / Writing
_CMD_EX_VAR   = 0x81  # Station/External Axis Variable (EX) Reading / Writing
_CMD_RESET      = 0x82  # Alarm Reset / Error Cancel
_CMD_HOLD_SERVO = 0x83  # HOLD / Servo ON-OFF / HLOCK
_CMD_CYCLE      = 0x84  # Step / Cycle / Auto switching
_CMD_MSG        = 0x85  # Character string display to pendant
_CMD_JOB_START  = 0x86  # Start-up (Job START)
_CMD_JOB_SELECT = 0x87  # Job select
_CMD_MGMT_TIME  = 0x88  # Management Time Acquiring
_CMD_SYS_INFO   = 0x89  # System Information Acquiring
_CMD_IO_PLURAL  = 0x300 # Plural I/O Data Reading / Writing
_CMD_REG_PLURAL = 0x301 # Plural Register Reading / Writing
_CMD_B_PLURAL   = 0x302 # Plural Byte (B) Reading / Writing
_CMD_I_PLURAL   = 0x303 # Plural Integer (I) Reading / Writing
_CMD_D_PLURAL   = 0x304 # Plural Double (D) Reading / Writing
_CMD_R_PLURAL   = 0x305 # Plural Real (R) Reading / Writing
_CMD_P_PLURAL   = 0x307 # Plural Robot Position (P) Reading / Writing
_CMD_BP_PLURAL  = 0x308 # Plural Base Position (BP) Reading / Writing
_CMD_EX_PLURAL  = 0x309 # Plural Station/External Axis (EX) Reading / Writing
_CMD_S_PLURAL   = 0x30C # Plural 32-Byte Character (S) Reading / Writing (DX200/YRC1000)
_CMD_S16_PLURAL = 0x306 # Plural 16-Byte Character (S) Reading / Writing (DX100)
_CMD_ALARM_SUB      = 0x30A # Alarm Data Reading (Sub-Code strings)
_CMD_ALARM_HIST_SUB = 0x30B # Alarm History Reading (Sub-Code strings)
_CMD_ENC_TEMP  = 0x0411 # Encoder Temperature Reading (SW YAS4.81+)
_CMD_REGEN_PWR = 0x0412 # Regenerative Power Reading (SW YAS4.90+)
_CMD_CONV_TEMP = 0x0413 # Converter Temperature Reading (SW YAS5.20+)

# Controller generations. They share almost every data layout; the differences are
# the S-variable command/width (DX100 = 16-byte 0x7E/0x306, newer = 32-byte
# 0x8C/0x30C), the meaning of status 0x72 Data1 bit4, and the YRC1000-only
# temperature commands (0x0411-0x0413). See manual HW1483358.
MODEL_DX100   = "dx100"
MODEL_DX200   = "dx200"
MODEL_YRC1000 = "yrc1000"
_VALID_MODELS = (MODEL_DX100, MODEL_DX200, MODEL_YRC1000)

_STATUS_CMD_UNDEFINED = 0x08  # "Requested command is undefined" (§4.1)
_CMD_MOVE_CART = 0x8A   # Move Instruction (Cartesian)
_CMD_MOVE_PULSE = 0x8B  # Move Instruction (Pulse)

# Service codes
_SVC_GET_ALL    = 0x01
_SVC_SET_ALL    = 0x02
_SVC_GET_SINGLE = 0x0E
_SVC_SET_SINGLE = 0x10
_SVC_READ_ALL   = 0x33   # plural read  (cmd 0x30x)
_SVC_WRITE_ALL  = 0x34   # plural write (cmd 0x30x)

# Instance selectors for HOLD / Servo command (0x83)
_HS_HOLD  = 1
_HS_SERVO = 2
_HS_HLOCK = 3

# ON/OFF data values shared by control commands
_ON  = 1
_OFF = 2

# Interpolation modes = instance selector for the move commands (0x8A/0x8B)
_MOVE_LINK_ABS     = 1   # joint interpolation to an absolute pose
_MOVE_STRAIGHT_ABS = 2   # linear interpolation to an absolute pose
_MOVE_STRAIGHT_INC = 3   # linear interpolation by an increment (0x8A only)

# Speed classification for the move commands (value 2 = VR 0.1 °/s is passed as a
# raw int on the speed_class field; see CartesianMove/PulseMove)
_SPEED_PERCENT = 0   # 0.01 % of max joint speed (link moves)
_SPEED_V       = 1   # 0.1 mm/s (straight moves)


@dataclass
class _RawResp:
    cmd:    int
    status: int    # 0x00 = normal
    added:  int    # added-status word (0 if none)
    data:   bytes  # raw payload


@dataclass
class PositionVar:
    """
    Robot position variable P[number] (cmd 0x7F) — a 52-byte record of 13 int32.

    `axes` holds the eight raw protocol coordinate values (unused axes are 0).
    For a cartesian data_type (16 Base / 17 Robot / 18 Tool / 19 User) the first
    six are X/Y/Z in µm and Rx/Ry/Rz in 0.0001°; for pulse (data_type 0) each is
    an encoder pulse count. Values are kept raw so a read → write round-trip is
    lossless.
    """
    data_type:       int              # 0=Pulse, 16=Base, 17=Robot, 18=Tool, 19=User
    figure:          int = 0          # configuration bits (arm flip/no-flip etc.)
    tool_no:         int = 0
    user_coord_no:   int = 0
    extended_figure: int = 0
    axes:            list = field(default_factory=lambda: [0] * 8)

    @classmethod
    def from_bytes(cls, data: bytes) -> "PositionVar":
        """Decode a 52-byte P record; short data is zero-padded to 13 int32."""
        n = len(data) // 4
        vals = list(struct.unpack_from(f"<{n}i", data)) if n else []
        vals += [0] * (13 - len(vals))
        return cls(
            data_type=vals[0], figure=vals[1], tool_no=vals[2],
            user_coord_no=vals[3], extended_figure=vals[4], axes=vals[5:13],
        )

    def to_bytes(self) -> bytes:
        """Encode to the 52-byte request payload (axes padded/validated to 8)."""
        axes = list(self.axes)
        if len(axes) > 8:
            raise ValueError("axes must have at most 8 elements")
        axes += [0] * (8 - len(axes))
        return struct.pack(
            "<13i", self.data_type, self.figure, self.tool_no,
            self.user_coord_no, self.extended_figure, *axes,
        )


@dataclass
class AxisPositionVar:
    """
    Base position (BP, cmd 0x80) or station/external axis (EX, cmd 0x81)
    variable — a 36-byte record of 9 int32: a data type plus eight axis values.

    For BP with a cartesian data_type (16) the first three axes are X/Y/Z in µm
    (base/positioner axes carry no orientation); data_type 0 (pulse) and every EX
    record carry raw encoder pulse counts. Values are kept raw so a read → write
    round-trip is lossless.
    """
    data_type: int              # BP: 0=Pulse, 16=Base;  EX: 0=Pulse only
    axes:      list = field(default_factory=lambda: [0] * 8)

    @classmethod
    def from_bytes(cls, data: bytes) -> "AxisPositionVar":
        """Decode a 36-byte BP/EX record; short data is zero-padded to 9 int32."""
        n = len(data) // 4
        vals = list(struct.unpack_from(f"<{n}i", data)) if n else []
        vals += [0] * (9 - len(vals))
        return cls(data_type=vals[0], axes=vals[1:9])

    def to_bytes(self) -> bytes:
        """Encode to the 36-byte request payload (axes padded/validated to 8)."""
        axes = list(self.axes)
        if len(axes) > 8:
            raise ValueError("axes must have at most 8 elements")
        axes += [0] * (8 - len(axes))
        return struct.pack("<9i", self.data_type, *axes)


def _pad(values, n: int, name: str) -> list:
    """Right-pad a raw-int list with zeros to exactly `n` (over-length is an error)."""
    vals = list(values)
    if len(vals) > n:
        raise ValueError(f"{name} must have at most {n} elements")
    return vals + [0] * (n - len(vals))


@dataclass
class CartesianMove:
    """
    Request record for a Cartesian move (cmd 0x8A) — 104 bytes = 26 int32.

    Coordinates are raw protocol units: X/Y/Z in µm, Rx/Ry/Rz in 0.0001°. A move
    targets EITHER the robot group OR a station group, never both (the controller
    rejects robot+station together with added-status 0xB008). Servo ON and Command
    Remote are required — this MOVES the robot.
    """
    x:  int = 0
    y:  int = 0
    z:  int = 0
    rx: int = 0
    ry: int = 0
    rz: int = 0
    speed_class:   int = _SPEED_V       # 0=% (link) · 1=V mm/s · 2=VR °/s
    speed:         int = 0              # link 0.01% · V 0.1 mm/s · VR 0.1°/s
    coord:         int = 17             # 16=Base 17=Robot 18=User 19=Tool
    robot_group:   int = 1              # 1-8 (0 = none, use station instead)
    station_group: int = 0              # 1-24 (0 = none)
    type:          int = 0              # pose configuration bitfield (like 0x75)
    expanded_type: int = 0
    tool_no:       int = 0              # 0-63
    user_coord_no: int = 0              # 1-63 (only when coord=18 User)
    base_axes:     list = field(default_factory=lambda: [0, 0, 0])   # µm, 3 axes
    station_axes:  list = field(default_factory=lambda: [0] * 6)     # pulse, 6 axes

    def to_bytes(self) -> bytes:
        """Encode to the 104-byte request payload in protocol field order."""
        base = _pad(self.base_axes, 3, "base_axes")
        station = _pad(self.station_axes, 6, "station_axes")
        return struct.pack(
            "<26i",
            self.robot_group, self.station_group, self.speed_class, self.speed,
            self.coord, self.x, self.y, self.z, self.rx, self.ry, self.rz,
            0, 0,                                    # reserve (offsets 44/48)
            self.type, self.expanded_type, self.tool_no, self.user_coord_no,
            *base, *station,
        )


@dataclass
class PulseMove:
    """
    Request record for a Pulse move (cmd 0x8B) — 88 bytes = 22 int32.

    `robot_axes` are raw encoder pulse counts (up to 8). Like the Cartesian move
    this targets EITHER the robot OR a station group (robot+station → 0xB008) and
    requires Servo ON + Command Remote. This MOVES the robot.
    """
    robot_axes:    list = field(default_factory=lambda: [0] * 8)     # pulse, 8 axes
    speed_class:   int = _SPEED_PERCENT   # 0=% · 1=V · 2=VR
    speed:         int = 0
    robot_group:   int = 1                # 1-8 (0 = none)
    station_group: int = 0                # 1-24 (0 = none)
    tool_no:       int = 0                # 0-63
    base_axes:     list = field(default_factory=lambda: [0, 0, 0])   # pulse, 3 axes
    station_axes:  list = field(default_factory=lambda: [0] * 6)     # pulse, 6 axes

    def to_bytes(self) -> bytes:
        """Encode to the 88-byte request payload in protocol field order."""
        robot = _pad(self.robot_axes, 8, "robot_axes")
        base = _pad(self.base_axes, 3, "base_axes")
        station = _pad(self.station_axes, 6, "station_axes")
        return struct.pack(
            "<22i",
            self.robot_group, self.station_group, self.speed_class, self.speed,
            *robot, self.tool_no, *base, *station,
        )


class HsesError(Exception):
    """Raised when the robot returns a non-zero status code."""
    def __init__(self, cmd: int, status: int, added: int) -> None:
        msg = (f"HSES error cmd=0x{cmd:04X} "
               f"status=0x{status:02X} ({status_text(status)})")
        if added:
            msg += f" added=0x{added:04X} ({added_status_text(added)})"
        super().__init__(msg)
        self.cmd    = cmd
        self.status = status
        self.added  = added


class _HsesSocket:
    """Shared UDP-transport lifecycle for the HSES telemetry and file clients:
    lazy socket open (optionally bound to a local adapter), idempotent close,
    context-manager support, and the 1..0xFF request-id rollover."""

    def __init__(self, host: str, port: int, timeout: float,
                 bind_host: Optional[str] = None) -> None:
        self._host      = host
        self._port      = port
        self._timeout   = timeout
        self._bind_host = bind_host   # local adapter IP to send from (optional)
        self._sock: Optional[socket.socket] = None
        self._req_id    = 1

    def open(self) -> None:
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self._bind_host:
            s.bind((self._bind_host, 0))   # pin egress to the chosen adapter
        s.settimeout(self._timeout)
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _bump_req_id(self) -> int:
        rid = self._req_id
        self._req_id = (self._req_id % 0xFF) + 1
        return rid


class HsesClient(_HsesSocket):
    """
    UDP client for the Yaskawa YRC1000 High-Speed Ethernet Server.

    Usage:
        with HsesClient("192.0.2.10") as c:
            print(c.read_status())
    """

    def __init__(self, host: str, port: int = HSES_PORT, timeout: float = 2.0,
                 bind_host: Optional[str] = None, model: str = MODEL_YRC1000) -> None:
        super().__init__(host, port, timeout, bind_host)
        self._model   = model
        self._axis_names: Optional[tuple] = None   # real 0x74 names, once loaded
        self._apply_model()   # derives S command/width from the model

    # ── controller model ───────────────────────────────────────────────────────

    def _apply_model(self) -> None:
        """Derive the model-dependent S-variable command and field width. DX100
        uses the 16-byte S variant (0x7E / 0x306); DX200 and YRC1000 the 32-byte
        one (0x8C / 0x30C)."""
        if self._model not in _VALID_MODELS:
            raise ValueError(f"unknown controller model: {self._model!r} "
                             f"(expected one of {_VALID_MODELS})")
        s16 = self._model == MODEL_DX100
        self._s_cmd        = _CMD_S16_VAR if s16 else _CMD_S_VAR
        self._s_plural_cmd = _CMD_S16_PLURAL if s16 else _CMD_S_PLURAL
        self._s_width      = 16 if s16 else 32

    @property
    def model(self) -> str:
        return self._model

    @model.setter
    def model(self, value: str) -> None:
        self._model = value
        self._apply_model()

    def detect_model(self) -> str:
        """Detect the controller generation with a capability probe (no version
        strings — those are not documented). Sends a 32-byte S read (0x8C): a
        DX100 rejects the command with status 0x08 ("undefined"), while DX200 /
        YRC1000 accept it. Sets self.model to the detected value and returns it.

        Cannot distinguish DX200 from YRC1000 (identical S behaviour) — it reports
        "yrc1000" for both; use an explicit model to get DX200's temperature-command
        guard. The status-0x08 assumption must be confirmed live against a DX100
        (docs/online_tests.md)."""
        try:
            self._tx(_CMD_S_VAR, instance=0, attribute=1, service=_SVC_GET_SINGLE)
            detected = MODEL_YRC1000
        except HsesError as e:
            detected = MODEL_DX100 if e.status == _STATUS_CMD_UNDEFINED else MODEL_YRC1000
        self.model = detected
        log.info("detect_model: probed 0x8C -> %s", detected)
        return detected

    def _require_yrc1000(self, what: str) -> None:
        """Guard for the YRC1000-only telemetry commands (0x0411-0x0413), which
        DX100/DX200 do not implement."""
        if self._model != MODEL_YRC1000:
            raise RuntimeError(
                f"{what} is only available on YRC1000 "
                f"(controller model is {self._model})")

    # ── axis names ─────────────────────────────────────────────────────────────
    @property
    def axis_names(self) -> tuple:
        """Per-axis labels used by the pulse-position / following-error / torque /
        encoder-temperature decoders. Returns the controller's real names once
        `load_axis_names()` has fetched them (e.g. S,L,U,T on a 4-axis palletiser),
        otherwise the S,L,U,R,B,T,E,8 default."""
        return self._axis_names or _AXIS_NAMES

    def load_axis_names(self, group: int = 1) -> tuple:
        """Fetch and cache the real axis names from Axis Configuration (0x74) so the
        per-axis reads are labelled with the controller's actual axis names instead
        of the S,L,U,R,B,T default (the 4th axis is R on a wrist robot but T on a
        palletiser). Empty slots keep a positional `axisN` label. Best-effort: on any
        error the cache is left unset and the default names apply. Call once after
        the connection is verified."""
        try:
            cfg = self.read_axis_config(group)
            names = tuple((cfg.get(f"axis_{i + 1}") or f"axis{i + 1}") for i in range(8))
            self._axis_names = names
            log.info("axis names from 0x74: %s", cfg.get("axes"))
        except Exception as e:  # noqa: BLE001 — best-effort, fall back to defaults
            log.warning("load_axis_names failed, using default names: %s", e)
        return self.axis_names

    # ── packet building ────────────────────────────────────────────────────────

    @staticmethod
    def _build(
        cmd: int, instance: int, attribute: int, service: int,
        req_id: int, payload: bytes = b"",
    ) -> bytes:
        hdr = bytearray(_HDR_SIZE)
        hdr[0:4] = _MAGIC
        struct.pack_into("<H", hdr,  4, _HDR_SIZE)    # header size (fixed 32)
        struct.pack_into("<H", hdr,  6, len(payload)) # data size
        hdr[8]  = 0x03                                 # Reserve1  FIXED
        hdr[9]  = _PROC_ROBOT
        hdr[10] = 0x00                                 # ACK = request
        hdr[11] = req_id
        struct.pack_into("<I", hdr, 12, 0)            # Block No.
        hdr[16:24] = b"99999999"                       # Reserve2  FIXED
        struct.pack_into("<H", hdr, 24, cmd)
        struct.pack_into("<H", hdr, 26, instance)
        hdr[28] = attribute
        hdr[29] = service
        struct.pack_into("<H", hdr, 30, 0)            # Padding
        return bytes(hdr) + payload

    # ── low-level exchange ────────────────────────────────────────────────────

    @staticmethod
    def _parse_resp(raw: bytes) -> _RawResp:
        """Decode a validated response header (magic/length already checked) into
        a _RawResp. In the answer header the Instance/Attribute/Padding fields are
        repurposed: bytes 24-25 → echoed command, 26 → status, 28 → added-status
        size, 30-31 → added-status word 1. Shared by _tx and _tx_multi."""
        dat_size    = struct.unpack_from("<H", raw, 6)[0]
        resp_status = raw[26]
        added_size  = raw[28]
        added       = struct.unpack_from("<H", raw, 30)[0] if added_size >= 1 else 0
        cmd_echo    = struct.unpack_from("<H", raw, 24)[0]
        data        = raw[_HDR_SIZE: _HDR_SIZE + dat_size]
        return _RawResp(cmd=cmd_echo, status=resp_status, added=added, data=data)

    def _tx(
        self,
        cmd:       int,
        instance:  int   = 1,
        attribute: int   = 0,
        service:   int   = _SVC_GET_ALL,
        payload:   bytes = b"",
    ) -> _RawResp:
        if self._sock is None:
            raise RuntimeError("Call open() first")

        rid = self._bump_req_id()
        pkt = self._build(cmd, instance, attribute, service, rid, payload)
        log.debug("tx  cmd=0x%04X inst=%d attr=%d svc=0x%02X id=%d payload=%dB",
                  cmd, instance, attribute, service, rid, len(payload))
        self._sock.sendto(pkt, (self._host, self._port))

        # Match the response by request ID (like _tx_multi). Without this
        # check, the late response of a previously timed-out request is read
        # as the response to this one — the offset then persists and silently
        # yields foreign payload data (e.g. bytes of a job name as the value
        # of a B variable).
        deadline = time.monotonic() + self._timeout
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    log.error("timeout cmd=0x%04X inst=%d id=%d after %.1fs",
                              cmd, instance, rid, self._timeout)
                    raise socket.timeout(
                        f"no response for cmd=0x{cmd:04X} id={rid}")
                self._sock.settimeout(remaining)
                try:
                    raw = self._sock.recv(4096)
                except socket.timeout:
                    log.error("timeout cmd=0x%04X inst=%d id=%d after %.1fs",
                              cmd, instance, rid, self._timeout)
                    raise
                if len(raw) < _HDR_SIZE:
                    log.error("malformed response cmd=0x%04X id=%d: too short (%d B)",
                              cmd, rid, len(raw))
                    raise IOError(f"Response too short: {len(raw)} B")
                if raw[0:4] != _MAGIC:
                    log.error("malformed response cmd=0x%04X id=%d: bad magic %r",
                              cmd, rid, raw[0:4])
                    raise IOError(f"Bad magic: {raw[0:4]!r}")
                if raw[11] != rid:
                    log.warning("verwerfe verspaetete Antwort id=%d (erwartet %d, "
                                "cmd=0x%04X)", raw[11], rid, cmd)
                    continue
                break
        finally:
            self._sock.settimeout(self._timeout)

        resp = self._parse_resp(raw)

        if resp.status != 0x00:
            log.error("rx  cmd=0x%04X id=%d status=0x%02X (%s) added=0x%04X (%s)",
                      cmd, rid, resp.status, status_text(resp.status),
                      resp.added, added_status_text(resp.added) if resp.added else "-")
            raise HsesError(cmd, resp.status, resp.added)

        log.debug("rx  cmd=0x%04X id=%d status=0 data=%dB", cmd, rid, len(resp.data))
        return resp

    # ── parallel burst ────────────────────────────────────────────────────────

    def _tx_multi(
        self,
        reqs: list[tuple],
    ) -> "list[Optional[_RawResp]]":
        """
        Send all requests simultaneously, collect responses matched by req_id.
        reqs: list of (cmd, instance, attribute, service, payload)
        Returns list of _RawResp|None in the same order as reqs.
        """
        if self._sock is None:
            raise RuntimeError("Call open() first")

        log.debug("tx_multi %d requests", len(reqs))
        req_ids: list[int] = []
        for cmd, instance, attribute, service, payload in reqs:
            rid = self._bump_req_id()
            req_ids.append(rid)
            pkt = self._build(cmd, instance, attribute, service, rid, payload)
            self._sock.sendto(pkt, (self._host, self._port))

        id_to_resp: dict[int, _RawResp] = {}
        pending = set(req_ids)
        deadline = time.monotonic() + self._timeout

        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._sock.settimeout(remaining)
            try:
                raw = self._sock.recv(4096)
            except OSError:
                break
            if len(raw) < _HDR_SIZE or raw[0:4] != _MAGIC:
                continue
            rid = raw[11]
            if rid not in pending:
                continue
            id_to_resp[rid] = self._parse_resp(raw)
            pending.discard(rid)

        self._sock.settimeout(self._timeout)
        if pending:
            log.warning("tx_multi: %d/%d responses missing (timeout)",
                        len(pending), len(req_ids))
        return [id_to_resp.get(rid) for rid in req_ids]

    def poll_all(
        self,
        *,
        alarm_slots: int = 0,
        include_job: bool = True,
        position_group: int = 0,
        b_vars: "list[dict]" = [],
        i_vars: "list[dict]" = [],
        io_signals: "list[dict]" = [],
        read_cio: bool = False,
    ) -> dict:
        """
        Fetch all configured data in one parallel UDP burst.

        alarm_slots:    number of alarm history slots (0=skip, max 4)
        include_job:    read executing job info
        position_group: 0=skip, 1-8=pulse, 101-108=cartesian
        b_vars/i_vars:  list of {"label": str, "no": int}
        io_signals:     list of {"label": str, "logical": int}

        Returns flat dict of all decoded fields.
        Raises HsesError if the STATUS command returns an error.
        """
        reqs: list[tuple] = []
        tags: list[str]   = []

        reqs.append((_CMD_STATUS, 1, 0, _SVC_GET_ALL, b""))
        tags.append("status")

        for slot in range(1, alarm_slots + 1):
            reqs.append((_CMD_ALARM, slot, 0, _SVC_GET_ALL, b""))
            tags.append(f"alarm{slot}")

        if include_job:
            reqs.append((_CMD_JOB, 1, 0, _SVC_GET_ALL, b""))
            tags.append("job")

        if position_group:
            reqs.append((_CMD_POSITION, position_group, 0, _SVC_GET_ALL, b""))
            tags.append("position")

        for v in b_vars:
            reqs.append((_CMD_B_VAR, v["no"], 1, _SVC_GET_SINGLE, b""))
            tags.append(f"b_var:{v['label']}")

        for v in i_vars:
            reqs.append((_CMD_I_VAR, v["no"], 1, _SVC_GET_SINGLE, b""))
            tags.append(f"i_var:{v['label']}")

        # CIO counter: low byte at logical 72000 (inst 7200), high byte at 72010 (inst 7201)
        if read_cio:
            reqs.append((_CMD_IO_PLURAL, 7200, 0, _SVC_READ_ALL, struct.pack("<I", 2)))
            tags.append("cio")

        # One plural read covers all io_signals regardless of count
        io_min_inst = 0
        if io_signals:
            io_min_inst = min(sig["logical"] // 10 for sig in io_signals)
            io_max_inst = max(sig["logical"] // 10 for sig in io_signals)
            n_bytes = io_max_inst - io_min_inst + 1
            if n_bytes % 2 != 0:
                n_bytes += 1
            reqs.append((_CMD_IO_PLURAL, io_min_inst, 0, _SVC_READ_ALL,
                          struct.pack("<I", n_bytes)))
            tags.append("io_plural")

        responses = self._tx_multi(reqs)
        out: dict = {}

        for tag, resp in zip(tags, responses):
            if resp is None:
                continue
            if tag == "status":
                if resp.status != 0:
                    raise HsesError(resp.cmd, resp.status, resp.added)
                out.update(_decode_status(resp.data, self._model))
            elif tag.startswith("alarm"):
                if resp.status == 0:
                    slot = int(tag[5:])
                    for k, v in _decode_alarm(resp.data).items():
                        out[f"alarm{slot}_{k[len('alarm_'):]}"] = v
            elif tag == "job":
                if resp.status == 0:
                    out.update(_decode_job(resp.data))
            elif tag == "position":
                if resp.status == 0:
                    out.update(_decode_position(resp.data))
            elif tag.startswith("b_var:"):
                if resp.status == 0 and len(resp.data) >= 1:
                    out[f"var_{tag[6:]}"] = resp.data[0]
            elif tag.startswith("i_var:"):
                if resp.status == 0 and len(resp.data) >= 2:
                    out[f"var_{tag[6:]}"] = struct.unpack_from("<h", resp.data, 0)[0]
            elif tag == "cio":
                if resp.status == 0 and len(resp.data) >= 6:
                    # response: 4-byte count + 2 data bytes
                    lo = resp.data[4]
                    hi = resp.data[5]
                    out["cio_counter"] = (hi << 8) | lo
            elif tag == "io_plural":
                if resp.status == 0 and len(resp.data) >= 4:
                    n = struct.unpack_from("<I", resp.data, 0)[0]
                    io_bytes = resp.data[4: 4 + n]
                    for sig in io_signals:
                        offset = sig["logical"] // 10 - io_min_inst
                        if offset < len(io_bytes):
                            out[f"io_{sig['label']}"] = io_bytes[offset]

        return out

    # ── public read methods ────────────────────────────────────────────────────

    def read_status(self) -> dict:
        """
        Status Information Reading (cmd 0x72).
        Returns dict with boolean flags:
            step, one_cycle, auto, running, speed_limit, teach, play, cmd_remote,
            hold_pendant, hold_external, hold_cmd, alarm, error, servo_on
        On DX100 the bit4 flag is keyed `in_guard_safe` instead of `speed_limit`.
        """
        r = self._tx(_CMD_STATUS, instance=1, attribute=0, service=_SVC_GET_ALL)
        if len(r.data) < 8:
            raise IOError(f"Status data too short: {len(r.data)} B")
        return _decode_status(r.data, self._model)

    def read_alarm(self, slot: int = 1) -> dict:
        """
        Alarm Data Reading (cmd 0x70).
        slot: 1=latest, 2=second from latest, 3=third, 4=fourth.
        Returns dict: alarm_code, alarm_data, alarm_type, alarm_time, alarm_name.
        alarm_type=0 means no alarm in that slot.
        """
        r = self._tx(_CMD_ALARM, instance=slot, attribute=0, service=_SVC_GET_ALL)
        return _decode_alarm(r.data)

    def read_alarm_history(self, number: int = 1) -> dict:
        """
        Alarm History Reading (cmd 0x71). Same 60-byte layout as read_alarm().
        `number` is the history slot per class:
            1-100    major alarms
            1001-1100 minor alarms
            2001-2100 user alarms (system)
            3001-3100 user alarms (user)
            4001-4100 off-line alarms
        Returns dict: alarm_code, alarm_data, alarm_type, alarm_time, alarm_name.
        alarm_type=0 (and code=0) means the slot is empty.
        """
        r = self._tx(_CMD_ALARM_HIST, instance=number, attribute=0,
                     service=_SVC_GET_ALL)
        return _decode_alarm(r.data)

    def read_alarm_sub(self, slot: int = 1) -> dict:
        """
        Alarm Data Reading with sub-code strings (cmd 0x30A). Extends
        read_alarm() with the diagnostic strings that appear next to the
        sub-code on the pendant.
        slot: 1=latest, 2=second from latest, 3=third, 4=fourth.
        Returns read_alarm()'s dict plus sub_code_info, sub_code_str and
        reverse_display. alarm_type=0 means no alarm in that slot.
        """
        r = self._tx(_CMD_ALARM_SUB, instance=slot, attribute=0,
                     service=_SVC_GET_ALL)
        return _decode_alarm_sub(r.data)

    def read_alarm_history_sub(self, number: int = 1) -> dict:
        """
        Alarm History Reading with sub-code strings (cmd 0x30B). Same extended
        268-byte layout as read_alarm_sub(); `number` is the history slot per
        class exactly as in read_alarm_history() (1-100 major, 1001+ minor,
        2001+/3001+ user, 4001+ off-line). Read-only.
        Returns the same dict as read_alarm_sub(); an empty slot has
        alarm_type=0 and alarm_code=0.
        """
        r = self._tx(_CMD_ALARM_HIST_SUB, instance=number, attribute=0,
                     service=_SVC_GET_ALL)
        return _decode_alarm_sub(r.data)

    def read_job(self, task: int = 1) -> dict:
        """
        Executing Job Information Reading (cmd 0x73).
        task: 1=master task, 2-16=sub tasks.
        Returns dict: job_name, job_line, job_step, job_speed.
        """
        r = self._tx(_CMD_JOB, instance=task, attribute=0, service=_SVC_GET_ALL)
        return _decode_job(r.data)

    def read_axis_config(self, group: int = 1) -> dict:
        """
        Axis Configuration Information Reading (cmd 0x74).
        group 1-8 = R1-R8 (pulse), 101-108 = R1-R8 (cartesian).
        Defines the axis order used by 0x75/0x76/0x77 (non-present axes are
        skipped, no gaps). Returns axis_1..axis_8 (name string per axis, empty
        if absent) plus 'axes' = list of present names. This robot: S,L,U,R.
        """
        r = self._tx(_CMD_AXIS_CFG, instance=group, attribute=0, service=_SVC_GET_ALL)
        return _decode_axis_config(r.data)

    def read_position(self, group: int = 1) -> dict:
        """
        Robot Position Data Reading (cmd 0x75).
        group 1-8  = R1-R8 pulse (joint angles in pulse units).
        group 101-108 = R1-R8 cartesian (X/Y/Z in mm, Rx/Ry/Rz in degrees).
        """
        r = self._tx(_CMD_POSITION, instance=group, attribute=0, service=_SVC_GET_ALL)
        return _decode_position(r.data, self.axis_names)

    def read_position_error(self, group: int = 1) -> dict:
        """
        Position Error (following error) Reading (cmd 0x76).
        group 1-8 = R1-R8. Returns the following error per axis in pulse units,
        keyed pe_S/pe_L/pe_U/pe_R/... in the axis order defined by 0x74.
        This 4-axis robot (S,L,U,R) leaves the remaining words at 0.
        """
        r = self._tx(_CMD_POSERR, instance=group, attribute=0, service=_SVC_GET_ALL)
        return _decode_axis_int32(r.data, "pe", self.axis_names)

    def read_torque(self, group: int = 1) -> dict:
        """
        Torque Data Reading (cmd 0x77).
        group 1-8 = R1-R8. Returns torque per axis (typ. % of rated torque,
        sign = direction), keyed tq_S/tq_L/tq_U/tq_R/... in the axis order
        defined by 0x74. This 4-axis robot leaves the remaining words at 0.
        """
        r = self._tx(_CMD_TORQUE, instance=group, attribute=0, service=_SVC_GET_ALL)
        return _decode_axis_int32(r.data, "tq", self.axis_names)

    def read_io(self, logical: int) -> int:
        """
        I/O Data Reading (cmd 0x78).
        logical: signal logical number as shown on the PP (e.g. 10010 = first output).
        instance = logical // 10.  Returns raw DWORD (0=OFF, non-zero=ON).
        """
        r = self._tx(_CMD_IO, instance=logical // 10, attribute=1, service=_SVC_GET_SINGLE)
        if not r.data:
            return 0
        return r.data[0]

    def read_io_plural(self, start_logical: int, count: int) -> bytes:
        """
        Plural I/O Data Reading (cmd 0x300, service 0x33).
        Reads `count` consecutive I/O bytes starting at start_logical in one request.
        count must be even, max 474.
        Returns raw bytes of length count.
        instance = start_logical // 10
        """
        if count % 2 != 0:
            count += 1  # must be even per spec
        payload = struct.pack("<I", count)
        r = self._tx(_CMD_IO_PLURAL, instance=start_logical // 10,
                     attribute=0, service=_SVC_READ_ALL, payload=payload)
        # Response data: DWORD 1 = count returned, then count bytes of I/O data
        if len(r.data) < 4:
            return b""
        n_returned = struct.unpack_from("<I", r.data, 0)[0]
        return r.data[4: 4 + n_returned]

    def write_io_plural(self, start_logical: int, data: bytes) -> None:
        """
        Plural I/O Data Writing (cmd 0x300, service 0x34).
        Writes len(data) consecutive I/O bytes starting at start_logical.
        len(data) must be even, max 474.
        instance = start_logical // 10
        Writable areas: network inputs (27010–29560), pseudo inputs (87010–87200).
        """
        count = len(data)
        if count % 2 != 0:
            data = data + b"\x00"
            count += 1
        payload = struct.pack("<I", count) + data
        self._tx(_CMD_IO_PLURAL, instance=start_logical // 10,
                 attribute=0, service=_SVC_WRITE_ALL, payload=payload)

    def write_io(self, logical: int, value: int) -> None:
        """
        Write a single I/O byte by logical address.
        value: 0–255 (one byte, 8 IO bits packed).
        Uses write_io_plural with 2 bytes (minimum even payload).
        """
        byte_val = value & 0xFF
        self.write_io_plural(logical, bytes([byte_val, 0x00]))

    # ── control commands ────────────────────────────────────────────────────────
    #
    # All of these need Command Remote enabled (RS 005 = 1) and, depending on the
    # action, the right mode / servo state.  _tx() raises HsesError on a non-zero
    # status, so a failed command (e.g. 0x2080 incorrect mode, 0x2100 command
    # remote not set) surfaces as an exception carrying the added-status code.

    def _control(self, cmd: int, instance: int, value: int) -> None:
        """Single-element control write (service 0x10) with a 4-byte int payload."""
        self._tx(cmd, instance=instance, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<I", value))

    def reset_alarm(self) -> None:
        """Alarm Reset (cmd 0x82, instance 1). Clears the active alarm."""
        log.info("control: reset_alarm")
        self._control(_CMD_RESET, 1, 1)

    def cancel_error(self) -> None:
        """Error Cancel (cmd 0x82, instance 2). Cancels the active error."""
        log.info("control: cancel_error")
        self._control(_CMD_RESET, 2, 1)

    def hold(self, on: bool = True) -> None:
        """HOLD (cmd 0x83, instance 1). on=True → hold, on=False → release."""
        log.info("control: hold %s", "on" if on else "off")
        self._control(_CMD_HOLD_SERVO, _HS_HOLD, _ON if on else _OFF)

    def servo(self, on: bool = True) -> None:
        """Servo power (cmd 0x83, instance 2). on=True → ON, on=False → OFF."""
        log.info("control: servo %s", "on" if on else "off")
        self._control(_CMD_HOLD_SERVO, _HS_SERVO, _ON if on else _OFF)

    def hlock(self, on: bool = True) -> None:
        """
        HLOCK (cmd 0x83, instance 3). Interlocks the programming pendant and I/O
        operation-system signals. on=True → lock, on=False → unlock.
        Invalid while the pendant is in edit mode or a file is being accessed.
        """
        log.info("control: hlock %s", "on" if on else "off")
        self._control(_CMD_HOLD_SERVO, _HS_HLOCK, _ON if on else _OFF)

    def set_cycle(self, mode: int) -> None:
        """
        Step / Cycle / Auto switching (cmd 0x84, instance 2).
        mode: 1=STEP, 2=1 CYCLE, 3=AUTO (continuous).
        """
        if mode not in (1, 2, 3):
            raise ValueError("cycle mode must be 1 (STEP), 2 (1 CYCLE) or 3 (AUTO)")
        log.info("control: set_cycle mode=%d", mode)
        self._control(_CMD_CYCLE, 2, mode)

    def display_message(self, text: str) -> None:
        """
        Character string display to the programming pendant (cmd 0x85).
        Up to 30 half-width characters (32-byte field, encoded in the pendant's
        language code — use latin-1 to match the YRC1000).
        """
        log.info("control: display_message %r", text)
        raw = text.encode("latin-1", errors="replace")[:32]
        payload = raw.ljust(32, b"\x00")
        self._tx(_CMD_MSG, instance=1, attribute=1, service=_SVC_SET_SINGLE,
                 payload=payload)

    def start_job(self) -> None:
        """
        Start-up / Job START (cmd 0x86, instance 1). Plays back the currently
        selected job. Requires servo ON and play mode.
        """
        log.info("control: start_job")
        self._control(_CMD_JOB_START, 1, 1)

    def select_job(self, name: str, line: int = 0, *,
                   master_task: Optional[int] = None) -> None:
        """
        Job Select (cmd 0x87, service 0x02 Set_All).
        name:        job name WITHOUT the .JBI extension (max 32 half-width chars).
        line:        start line number, 0-9999.
        master_task: None → set the executing job (instance 1).
                     0-15 → set the master job for that task (instance 10-25).

        Attribute is 0 (Set_All targets element 0 = the whole record). The robot
        rejects a ".JBI" suffix and a non-zero attribute with status 0x01
        (verified live 2026-07-04).
        """
        if master_task is None:
            instance = 1
        elif 0 <= master_task <= 15:
            instance = 10 + master_task
        else:
            raise ValueError("master_task must be 0-15 or None")
        if not 0 <= line <= 9999:
            raise ValueError("line must be 0-9999")
        log.info("control: select_job name=%r line=%d task=%s",
                 name, line, master_task)
        raw = name.encode("latin-1", errors="replace")[:32].ljust(32, b"\x00")
        payload = raw + struct.pack("<I", line)
        self._tx(_CMD_JOB_SELECT, instance=instance, attribute=0,
                 service=_SVC_SET_ALL, payload=payload)

    # ── motion commands (⚠️ move the robot) ──────────────────────────────────────
    #
    # These order a real motion: Servo ON, Command Remote (RS 005 = 1) and PLAY
    # mode are required, and the controller executes the move immediately. Only
    # use them with a clear work envelope and the emergency stop within reach.

    def move_cartesian(self, move: CartesianMove,
                       mode: int = _MOVE_STRAIGHT_ABS) -> None:
        """
        Cartesian move (cmd 0x8A). ⚠️ MOVES the robot to the pose in `move`.

        mode: 1=link-absolute (joint interp.), 2=straight-absolute (linear),
              3=straight-increment (linear, relative to the current pose).
        Attribute is fixed 1, service is Set_All (0x02, execute).
        """
        if mode not in (_MOVE_LINK_ABS, _MOVE_STRAIGHT_ABS, _MOVE_STRAIGHT_INC):
            raise ValueError("mode must be 1 (link), 2 (straight) or 3 (increment)")
        log.info("control: move_cartesian mode=%d coord=%d group=r%d/s%d",
                 mode, move.coord, move.robot_group, move.station_group)
        self._tx(_CMD_MOVE_CART, instance=mode, attribute=1,
                 service=_SVC_SET_ALL, payload=move.to_bytes())

    def move_pulse(self, move: PulseMove,
                   mode: int = _MOVE_LINK_ABS) -> None:
        """
        Pulse move (cmd 0x8B). ⚠️ MOVES the robot to the joint pulses in `move`.

        mode: 1=link-absolute (joint interp.), 2=straight-absolute (linear).
        Attribute is fixed 1, service is Set_All (0x02, execute).
        """
        if mode not in (_MOVE_LINK_ABS, _MOVE_STRAIGHT_ABS):
            raise ValueError("mode must be 1 (link) or 2 (straight)")
        log.info("control: move_pulse mode=%d group=r%d/s%d",
                 mode, move.robot_group, move.station_group)
        self._tx(_CMD_MOVE_PULSE, instance=mode, attribute=1,
                 service=_SVC_SET_ALL, payload=move.to_bytes())

    # ── variable reads ──────────────────────────────────────────────────────────

    def read_register(self, number: int) -> int:
        """
        Register M[number] (cmd 0x79). Registers are 16-bit values carried in a
        4-byte field; returns 0-65535. Instance is the register number 0-999.
        """
        r = self._tx(_CMD_REGISTER, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        if len(r.data) < 2:
            return 0
        return struct.unpack_from("<H", r.data, 0)[0]

    def read_b_var(self, number: int) -> int:
        """Byte Variable B[number] (cmd 0x7A). Returns 0-255."""
        r = self._tx(_CMD_B_VAR, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        if len(r.data) < 1:
            return 0
        return r.data[0]

    def read_i_var(self, number: int) -> int:
        """Integer Variable I[number] (cmd 0x7B). Returns the signed 16-bit value
        carried in the low word of the 4-byte slot (-32768..32767); use D (0x7C)
        for 32-bit. Matches write_i_var / the plural 0x303 path."""
        r = self._tx(_CMD_I_VAR, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        if len(r.data) < 2:
            return 0
        return struct.unpack_from("<h", r.data, 0)[0]

    def read_d_var(self, number: int) -> int:
        """Double Variable D[number] (cmd 0x7C). Returns signed 32-bit int."""
        r = self._tx(_CMD_D_VAR, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        if len(r.data) < 4:
            return 0
        return struct.unpack_from("<i", r.data, 0)[0]

    def read_r_var(self, number: int) -> float:
        """Real Variable R[number] (cmd 0x7D). Returns IEEE-754 float32."""
        r = self._tx(_CMD_R_VAR, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        if len(r.data) < 4:
            return 0.0
        return struct.unpack_from("<f", r.data, 0)[0]

    def read_s_var(self, number: int) -> str:
        """
        Character Variable S[number]. 32-byte cmd 0x8C on DX200/YRC1000, 16-byte
        cmd 0x7E on DX100 (selected by the client model). Returns the string with
        trailing NULs stripped, decoded as latin-1 to match the pendant's language.
        """
        r = self._tx(self._s_cmd, instance=number, attribute=1, service=_SVC_GET_SINGLE)
        return r.data[:self._s_width].split(b"\x00", 1)[0].decode("latin-1")

    # ── variable writes ─────────────────────────────────────────────────────────

    def write_register(self, number: int, value: int) -> None:
        """
        Write Register M[number] (cmd 0x79, service 0x10 Set_Single).
        value: 0-65535, carried in the low word of a 4-byte slot. Only registers
        below the writable limit (0-559, controller-dependent) accept a write;
        higher numbers are read-only and reject with an HsesError.
        """
        if not 0 <= value <= 65535:
            raise ValueError("register value must be 0-65535")
        self._tx(_CMD_REGISTER, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<I", value))

    def write_b_var(self, number: int, value: int) -> None:
        """
        Write Byte Variable B[number] (cmd 0x7A, service 0x10 Set_Single).
        value: 0-255. Payload is a 4-byte slot with the value in the low byte.
        """
        if not 0 <= value <= 255:
            raise ValueError("B variable value must be 0-255")
        self._tx(_CMD_B_VAR, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<I", value))

    def write_i_var(self, number: int, value: int) -> None:
        """
        Write Integer Variable I[number] (cmd 0x7B, service 0x10 Set_Single).
        value: signed 16-bit (-32768..32767), carried in the low word of a
        4-byte slot.
        """
        if not -32768 <= value <= 32767:
            raise ValueError("I variable value must be a signed 16-bit int")
        self._tx(_CMD_I_VAR, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<i", value))

    def write_d_var(self, number: int, value: int) -> None:
        """
        Write Double Variable D[number] (cmd 0x7C, service 0x10 Set_Single).
        value: signed 32-bit int (-2147483648..2147483647).
        """
        if not -2_147_483_648 <= value <= 2_147_483_647:
            raise ValueError("D variable value must be a signed 32-bit int")
        self._tx(_CMD_D_VAR, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<i", value))

    def write_r_var(self, number: int, value: float) -> None:
        """
        Write Real Variable R[number] (cmd 0x7D, service 0x10 Set_Single).
        value: IEEE-754 float32.
        """
        self._tx(_CMD_R_VAR, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=struct.pack("<f", value))

    def write_s_var(self, number: int, value: str) -> None:
        """
        Write Character Variable S[number] (service 0x10). 32-byte cmd 0x8C on
        DX200/YRC1000, 16-byte cmd 0x7E on DX100. value is encoded as latin-1
        (pendant language code), truncated and NUL-padded to the field width.
        """
        raw = value.encode("latin-1", errors="replace")[:self._s_width]
        self._tx(self._s_cmd, instance=number, attribute=1, service=_SVC_SET_SINGLE,
                 payload=raw.ljust(self._s_width, b"\x00"))

    def read_position_var(self, number: int) -> PositionVar:
        """
        Robot Position Variable P[number] (cmd 0x7F, service 0x01 Get_All).
        Instance is the variable number 0-127. P only supports "All" access, so
        the whole 52-byte record is returned as a PositionVar.
        """
        r = self._tx(_CMD_P_VAR, instance=number, attribute=0, service=_SVC_GET_ALL)
        return PositionVar.from_bytes(r.data)

    def write_position_var(self, number: int, pos: PositionVar) -> None:
        """
        Write Robot Position Variable P[number] (cmd 0x7F, service 0x02 Set_All).
        P only supports "All" access, so the whole 52-byte record is written at
        once from the given PositionVar.
        """
        self._tx(_CMD_P_VAR, instance=number, attribute=0, service=_SVC_SET_ALL,
                 payload=pos.to_bytes())

    def read_bp_var(self, number: int) -> AxisPositionVar:
        """
        Base Position Variable BP[number] (cmd 0x80, service 0x01 Get_All).
        Instance is the variable number 0-127. Returns the whole 36-byte record
        as an AxisPositionVar (data_type 0=Pulse or 16=Base cartesian).
        """
        r = self._tx(_CMD_BP_VAR, instance=number, attribute=0, service=_SVC_GET_ALL)
        return AxisPositionVar.from_bytes(r.data)

    def write_bp_var(self, number: int, pos: AxisPositionVar) -> None:
        """
        Write Base Position Variable BP[number] (cmd 0x80, service 0x02 Set_All).
        Only data_type 0 (Pulse) and 16 (Base cartesian) are valid.
        """
        self._tx(_CMD_BP_VAR, instance=number, attribute=0, service=_SVC_SET_ALL,
                 payload=pos.to_bytes())

    def read_ex_var(self, number: int) -> AxisPositionVar:
        """
        Station/External Axis Variable EX[number] (cmd 0x81, service 0x01 Get_All).
        Instance is the variable number 0-127. EX only carries pulse counts
        (data_type 0); returns the 36-byte record as an AxisPositionVar.
        """
        r = self._tx(_CMD_EX_VAR, instance=number, attribute=0, service=_SVC_GET_ALL)
        return AxisPositionVar.from_bytes(r.data)

    def write_ex_var(self, number: int, pos: AxisPositionVar) -> None:
        """
        Write Station/External Axis Variable EX[number] (cmd 0x81, service 0x02
        Set_All). EX is pulse-only (data_type 0).
        """
        self._tx(_CMD_EX_VAR, instance=number, attribute=0, service=_SVC_SET_ALL,
                 payload=pos.to_bytes())

    # ── plural variable reads / writes ────────────────────────────────────────────
    #
    # Plural commands (0x301–0x309, 0x30C) read/write a run of consecutive
    # variables in a single request. Attribute is fixed 0, service 0x33 reads /
    # 0x34 writes. The request carries a uint32 element count; a read response
    # echoes the count returned, followed by the packed elements. Instance is the
    # start variable number. Per-command element sizes and max counts are in
    # HW1483358 §3.3.3.28–36/42. The 16-byte S variant (0x306) is
    # skipped in favour of the 32-byte 0x30C, mirroring the scalar 0x8C choice.

    def _read_plural(self, cmd: int, start: int, count: int) -> bytes:
        """Send a plural read (service 0x33) and return the raw element bytes,
        trimmed to the element count the controller reports it returned."""
        r = self._tx(cmd, instance=start, attribute=0, service=_SVC_READ_ALL,
                     payload=struct.pack("<I", count))
        if len(r.data) < 4:
            return b""
        return r.data[4:]

    def _write_plural(self, cmd: int, start: int, count: int, data: bytes) -> None:
        """Send a plural write (service 0x34): a uint32 element count then the
        packed element bytes, starting at variable number `start`."""
        self._tx(cmd, instance=start, attribute=0, service=_SVC_WRITE_ALL,
                 payload=struct.pack("<I", count) + data)

    def _read_plural_scalar(self, cmd: int, start: int, count: int,
                            fmt: str, size: int) -> list:
        """Plural read of fixed-width scalar elements (`fmt`/`size` per struct),
        empty-safe. Shared by the register/I/D/R variable reads."""
        buf = self._read_plural(cmd, start, count)
        n = len(buf) // size
        return list(struct.unpack_from(f"<{n}{fmt}", buf)) if n else []

    def read_registers(self, start: int, count: int) -> "list[int]":
        """
        Plural Register read (cmd 0x301). Reads `count` consecutive registers
        starting at register number `start`. Each register is an unsigned 16-bit
        value (2-byte element); returns a list of 0-65535 ints (max 237).
        """
        return self._read_plural_scalar(_CMD_REG_PLURAL, start, count, "H", 2)

    def write_registers(self, start: int, values: "list[int]") -> None:
        """
        Plural Register write (cmd 0x301). Writes `values` to consecutive
        registers from `start`. Each value 0-65535 (2-byte element). Only
        registers below the writable limit (0-559) accept a write.
        """
        for v in values:
            if not 0 <= v <= 65535:
                raise ValueError("register value must be 0-65535")
        self._write_plural(_CMD_REG_PLURAL, start, len(values),
                           struct.pack(f"<{len(values)}H", *values))

    def read_b_vars(self, start: int, count: int) -> "list[int]":
        """
        Plural Byte Variable read (cmd 0x302). Returns `count` consecutive B
        variables from `start` as a list of 0-255 ints (max 474). The wire count
        must be even, so an odd `count` reads one extra element and trims it.
        """
        buf = self._read_plural(_CMD_B_PLURAL, start, count + (count & 1))
        return list(buf[:count])

    def write_b_vars(self, start: int, values: "list[int]") -> None:
        """
        Plural Byte Variable write (cmd 0x302). Writes `values` (each 0-255) to
        consecutive B variables from `start`. The wire count must be even, so an
        odd number of values is rejected — split or pad the run explicitly.
        """
        if len(values) % 2 != 0:
            raise ValueError("B plural write needs an even number of values")
        for v in values:
            if not 0 <= v <= 255:
                raise ValueError("B variable value must be 0-255")
        self._write_plural(_CMD_B_PLURAL, start, len(values), bytes(values))

    def read_i_vars(self, start: int, count: int) -> "list[int]":
        """
        Plural Integer Variable read (cmd 0x303). Returns `count` consecutive I
        variables from `start` as signed 16-bit ints (2-byte element, max 237).
        """
        return self._read_plural_scalar(_CMD_I_PLURAL, start, count, "h", 2)

    def write_i_vars(self, start: int, values: "list[int]") -> None:
        """
        Plural Integer Variable write (cmd 0x303). Writes `values` (each a signed
        16-bit int) to consecutive I variables from `start`.
        """
        for v in values:
            if not -32768 <= v <= 32767:
                raise ValueError("I variable value must be a signed 16-bit int")
        self._write_plural(_CMD_I_PLURAL, start, len(values),
                           struct.pack(f"<{len(values)}h", *values))

    def read_d_vars(self, start: int, count: int) -> "list[int]":
        """
        Plural Double Variable read (cmd 0x304). Returns `count` consecutive D
        variables from `start` as signed 32-bit ints (4-byte element, max 118).
        """
        return self._read_plural_scalar(_CMD_D_PLURAL, start, count, "i", 4)

    def write_d_vars(self, start: int, values: "list[int]") -> None:
        """
        Plural Double Variable write (cmd 0x304). Writes `values` (each a signed
        32-bit int) to consecutive D variables from `start`.
        """
        for v in values:
            if not -2_147_483_648 <= v <= 2_147_483_647:
                raise ValueError("D variable value must be a signed 32-bit int")
        self._write_plural(_CMD_D_PLURAL, start, len(values),
                           struct.pack(f"<{len(values)}i", *values))

    def read_r_vars(self, start: int, count: int) -> "list[float]":
        """
        Plural Real Variable read (cmd 0x305). Returns `count` consecutive R
        variables from `start` as IEEE-754 float32 (4-byte element, max 118).
        """
        return self._read_plural_scalar(_CMD_R_PLURAL, start, count, "f", 4)

    def write_r_vars(self, start: int, values: "list[float]") -> None:
        """
        Plural Real Variable write (cmd 0x305). Writes `values` as IEEE-754
        float32 to consecutive R variables from `start`.
        """
        self._write_plural(_CMD_R_PLURAL, start, len(values),
                           struct.pack(f"<{len(values)}f", *values))

    def read_s_vars(self, start: int, count: int) -> "list[str]":
        """
        Plural Character Variable read. 32-byte cmd 0x30C on DX200/YRC1000
        (max 14), 16-byte cmd 0x306 on DX100 (max 29). Returns `count` consecutive
        S variables from `start` as strings (trailing NULs stripped, latin-1).
        """
        w = self._s_width
        buf = self._read_plural(self._s_plural_cmd, start, count)
        n = len(buf) // w
        return [buf[i * w:i * w + w].split(b"\x00", 1)[0].decode("latin-1")
                for i in range(n)]

    def write_s_vars(self, start: int, values: "list[str]") -> None:
        """
        Plural Character Variable write. 32-byte cmd 0x30C on DX200/YRC1000,
        16-byte cmd 0x306 on DX100. Writes `values` to consecutive S variables
        from `start`; each is latin-1 encoded, truncated and NUL-padded to width.
        """
        w = self._s_width
        data = b"".join(
            v.encode("latin-1", errors="replace")[:w].ljust(w, b"\x00")
            for v in values
        )
        self._write_plural(self._s_plural_cmd, start, len(values), data)

    def read_position_vars(self, start: int, count: int) -> "list[PositionVar]":
        """
        Plural Robot Position Variable read (cmd 0x307). Returns `count`
        consecutive P variables from `start` as PositionVar records (52-byte
        element, max 9).
        """
        buf = self._read_plural(_CMD_P_PLURAL, start, count)
        n = len(buf) // 52
        return [PositionVar.from_bytes(buf[i * 52:i * 52 + 52]) for i in range(n)]

    def write_position_vars(self, start: int, values: "list[PositionVar]") -> None:
        """
        Plural Robot Position Variable write (cmd 0x307). Writes `values` to
        consecutive P variables from `start`.
        """
        self._write_plural(_CMD_P_PLURAL, start, len(values),
                           b"".join(p.to_bytes() for p in values))

    def _read_axispos_plural(self, cmd: int, start: int, count: int) -> "list[AxisPositionVar]":
        """Plural read of 36-byte AxisPositionVar records. Shared by BP and EX."""
        buf = self._read_plural(cmd, start, count)
        n = len(buf) // 36
        return [AxisPositionVar.from_bytes(buf[i * 36:i * 36 + 36])
                for i in range(n)]

    def read_bp_vars(self, start: int, count: int) -> "list[AxisPositionVar]":
        """
        Plural Base Position Variable read (cmd 0x308). Returns `count`
        consecutive BP variables from `start` as AxisPositionVar records
        (36-byte element, max 13).
        """
        return self._read_axispos_plural(_CMD_BP_PLURAL, start, count)

    def write_bp_vars(self, start: int, values: "list[AxisPositionVar]") -> None:
        """
        Plural Base Position Variable write (cmd 0x308). Writes `values` to
        consecutive BP variables from `start`.
        """
        self._write_plural(_CMD_BP_PLURAL, start, len(values),
                           b"".join(p.to_bytes() for p in values))

    def read_ex_vars(self, start: int, count: int) -> "list[AxisPositionVar]":
        """
        Plural Station/External Axis Variable read (cmd 0x309). Returns `count`
        consecutive EX variables from `start` as AxisPositionVar records
        (36-byte element, max 13; pulse-only).
        """
        return self._read_axispos_plural(_CMD_EX_PLURAL, start, count)

    def write_ex_vars(self, start: int, values: "list[AxisPositionVar]") -> None:
        """
        Plural Station/External Axis Variable write (cmd 0x309). Writes `values`
        to consecutive EX variables from `start` (pulse-only).
        """
        self._write_plural(_CMD_EX_PLURAL, start, len(values),
                           b"".join(p.to_bytes() for p in values))

    # ── system / management info ──────────────────────────────────────────────────

    def read_management_time(self, kind: int = 1) -> dict:
        """
        Management Time Acquiring (cmd 0x88).
        kind (instance): 1=Control-Power-ON, 10=Servo-Power-ON total, 11-18=Servo
        R1-R8, 110=Playback total, 210=Moving total, 301-308=Operation app 1-8.
        Returns dict: start_time (e.g. "2011/10/10 15:49"), elapse_time
        (e.g. "000000:00'00").
        """
        r = self._tx(_CMD_MGMT_TIME, instance=kind, attribute=0, service=_SVC_GET_ALL)
        return _decode_management_time(r.data)

    def read_system_info(self, kind: int = 11) -> dict:
        """
        System Information Acquiring (cmd 0x89).
        kind (instance): 11-18=Type R1-R8, 21-44=Type S1-S24, 101-108=Application.
        Returns dict: sw_version (e.g. "YAS1.00.00A. (JP/US) -00"), model,
        parameter_version.
        """
        r = self._tx(_CMD_SYS_INFO, instance=kind, attribute=0, service=_SVC_GET_ALL)
        return _decode_system_info(r.data)

    def read_encoder_temp(self, group: int = 1) -> dict:
        """
        Encoder Temperature Reading (cmd 0x0411, controller SW YAS4.81+).
        group 1-8 = R1-R8. Returns encoder temperature per axis in °C, keyed
        enc_temp_S/enc_temp_L/... (axis order per 0x74).
        Raises HsesError if the firmware predates YAS4.81, or RuntimeError on a
        DX100/DX200 controller (command not implemented).
        """
        self._require_yrc1000("Encoder temperature (0x0411)")
        r = self._tx(_CMD_ENC_TEMP, instance=group, attribute=1, service=_SVC_GET_ALL)
        return _decode_axis_int32(r.data, "enc_temp", self.axis_names)

    def read_regen_power(self) -> dict:
        """
        Regenerative Power Reading (cmd 0x0412, controller SW YAS4.90+).
        Returns dict: valid_boards (bitmask), status (0=servo-off, 1=consumption,
        2=regeneration), unit (0=kWh, 1=0.01 kWh) and boards[] each with
        today/yesterday/cumulative energy.
        Raises HsesError if the firmware predates YAS4.90, or RuntimeError on a
        DX100/DX200 controller (command not implemented).
        """
        self._require_yrc1000("Regenerative power (0x0412)")
        r = self._tx(_CMD_REGEN_PWR, instance=1, attribute=1, service=_SVC_GET_ALL)
        return _decode_regen_power(r.data)

    def read_converter_temp(self, board: int = 1) -> int:
        """
        Converter Temperature Reading (cmd 0x0413, controller SW YAS5.20+).
        board 1-8 = servo board number. Returns converter temperature in °C.
        Raises HsesError if the firmware predates YAS5.20, or RuntimeError on a
        DX100/DX200 controller (command not implemented).
        """
        self._require_yrc1000("Converter temperature (0x0413)")
        r = self._tx(_CMD_CONV_TEMP, instance=board, attribute=1, service=_SVC_GET_ALL)
        if len(r.data) < 4:
            return 0
        return struct.unpack_from("<i", r.data, 0)[0]


# ── response decoders ─────────────────────────────────────────────────────────

def _decode_status(data: bytes, model: str = MODEL_YRC1000) -> dict:
    if len(data) < 8:
        return {}
    d1 = struct.unpack_from("<I", data, 0)[0]
    d2 = struct.unpack_from("<I", data, 4)[0]
    # Data1 bit4 differs by generation: DX100 = "in-guard safe operation",
    # DX200/YRC1000 = "speed limit". Emit the model-appropriate key.
    bit4_key = "in_guard_safe" if model == MODEL_DX100 else "speed_limit"
    return {
        "step":          bool(d1 & 0x01),
        "one_cycle":     bool(d1 & 0x02),
        "auto":          bool(d1 & 0x04),
        "running":       bool(d1 & 0x08),
        bit4_key:        bool(d1 & 0x10),
        "teach":         bool(d1 & 0x20),
        "play":          bool(d1 & 0x40),
        "cmd_remote":    bool(d1 & 0x80),
        "hold_pendant":  bool(d2 & 0x02),
        "hold_external": bool(d2 & 0x04),
        "hold_cmd":      bool(d2 & 0x08),
        "alarm":         bool(d2 & 0x10),
        "error":         bool(d2 & 0x20),
        "servo_on":      bool(d2 & 0x40),
    }


def _decode_alarm(data: bytes) -> dict:
    """Decode alarm data (up to 60 bytes = 15 × 32-bit LE ints, Section 3.3.3.1)."""
    out: dict = {
        "alarm_code": 0, "alarm_data": 0, "alarm_type": 0,
        "alarm_time": "", "alarm_name": "",
    }
    if len(data) < 12:
        return out
    out["alarm_code"] = struct.unpack_from("<I", data,  0)[0]
    out["alarm_data"] = struct.unpack_from("<I", data,  4)[0]
    out["alarm_type"] = struct.unpack_from("<I", data,  8)[0]
    if len(data) >= 28:
        # integers 4-7 = alarm time, 16 chars
        out["alarm_time"] = data[12:28].rstrip(b"\x00").decode("latin-1", errors="replace")
    if len(data) >= 60:
        # integers 8-15 = alarm name, 32 chars
        out["alarm_name"] = data[28:60].rstrip(b"\x00").decode("latin-1", errors="replace")
    return out


def _decode_alarm_sub(data: bytes) -> dict:
    """
    Decode extended alarm data with sub-code strings (0x30A/0x30B, up to 268
    bytes, Section 3.3.3.37). First 60 bytes are identical to _decode_alarm();
    the three trailing char fields carry the diagnostic sub-code strings:
        [60] char[16] sub-code extra info (e.g. "[SV#1]")
        [76] char[96] sub-code data string
        [172] char[96] reverse-display info ("0"=normal, "1"=inverted)
    """
    out = _decode_alarm(data)
    out.update({"sub_code_info": "", "sub_code_str": "", "reverse_display": ""})
    if len(data) >= 76:
        out["sub_code_info"] = _cstr(data[60:76])
    if len(data) >= 172:
        out["sub_code_str"] = _cstr(data[76:172])
    if len(data) >= 268:
        out["reverse_display"] = _cstr(data[172:268])
    return out


def _decode_job(data: bytes) -> dict:
    """Decode executing job data (44 bytes = 11 × 32-bit LE ints, Section 3.3.3.4)."""
    out: dict = {"job_name": "", "job_line": 0, "job_step": 0, "job_speed": 0}
    if len(data) < 32:
        return out
    out["job_name"] = data[0:32].rstrip(b"\x00").decode("latin-1", errors="replace")
    if len(data) >= 44:
        line, step, speed = struct.unpack_from("<3I", data, 32)
        out["job_line"]  = line
        out["job_step"]  = step
        out["job_speed"] = speed
    return out


def _cstr(raw: bytes) -> str:
    """Decode a fixed-width, null/space-padded ASCII field to a stripped str."""
    return raw.decode("latin-1", errors="replace").split("\x00", 1)[0].strip()


def _decode_regen_power(data: bytes) -> dict:
    """
    Decode regenerative power (0x0412, 104 bytes = 26 × int32): word0 valid-board
    bitmask, word1 byte-packed status/unit, then per board (1-8) three int32s
    today / yesterday / cumulative.
    """
    out: dict = {"valid_boards": 0, "status": 0, "unit": 0, "boards": []}
    if len(data) < 8:
        return out
    out["valid_boards"] = struct.unpack_from("<I", data, 0)[0]
    out["status"] = data[4]   # 0=servo-off, 1=consumption, 2=regeneration
    out["unit"] = data[5]     # 0=kWh, 1=0.01 kWh
    for b in range(8):
        off = 8 + b * 12
        if off + 12 > len(data):
            break
        today, yesterday, cumulative = struct.unpack_from("<3i", data, off)
        out["boards"].append({
            "board": b + 1,
            "valid": bool(out["valid_boards"] & (1 << b)),
            "today": today,
            "yesterday": yesterday,
            "cumulative": cumulative,
        })
    return out


def _decode_management_time(data: bytes) -> dict:
    """Decode management time (0x88): char[16] start + char[12] elapse."""
    out = {"start_time": "", "elapse_time": ""}
    if len(data) >= 16:
        out["start_time"] = _cstr(data[0:16])
    if len(data) >= 28:
        out["elapse_time"] = _cstr(data[16:28])
    return out


def _decode_system_info(data: bytes) -> dict:
    """Decode system info (0x89): char[24] sw + char[16] model + char[8] param."""
    out = {"sw_version": "", "model": "", "parameter_version": ""}
    if len(data) >= 24:
        out["sw_version"] = _cstr(data[0:24])
    if len(data) >= 40:
        out["model"] = _cstr(data[24:40])
    if len(data) >= 48:
        out["parameter_version"] = _cstr(data[40:48])
    return out


def _decode_axis_config(data: bytes) -> dict:
    """
    Decode axis configuration (32 bytes = 8 × 4-byte words, 0x74). Each word
    holds an axis name as ASCII characters (LE, null-padded); 0 = axis absent.
    """
    out: dict = {"axes": []}
    n = min(len(data) // 4, 8)
    for i in range(n):
        name = data[i * 4:i * 4 + 4].rstrip(b"\x00").decode("latin-1", errors="replace")
        out[f"axis_{i + 1}"] = name
        if name:
            out["axes"].append(name)
    return out


_AXIS_NAMES = ("S", "L", "U", "R", "B", "T", "E", "8")


def _decode_axis_int32(data: bytes, prefix: str, names: tuple = _AXIS_NAMES) -> dict:
    """
    Decode 8 × 32-bit LE signed ints as per-axis values (0x76 position error,
    0x77 torque, 0x0411 encoder temp). Keys are f"{prefix}_{axis}" in the axis
    order defined by 0x74 — pass the client's `axis_names` to label the 4th axis
    correctly (R on a wrist robot, T on a palletiser); defaults to S,L,U,R,B,T,E,8.
    """
    out: dict = {}
    n = min(len(data) // 4, len(names))
    if n == 0:
        return out
    vals = struct.unpack_from(f"<{n}i", data)
    for name, v in zip(names, vals):
        out[f"{prefix}_{name}"] = v
    return out


def _decode_position(data: bytes, names: Optional[tuple] = None) -> dict:
    """Decode position data (52 bytes = 13 × 32-bit LE signed ints, Section 3.3.3.6).

    For a pulse record the joint values are labelled with the axis order from 0x74
    (pass the client's `axis_names`); without it the S,L,U,R,B,T default applies."""
    out: dict = {}
    n = len(data) // 4
    if n < 6:
        return out
    vals = list(struct.unpack_from(f"<{n}i", data))
    data_type = vals[0]   # 0=pulse, 16=cartesian (base coordinate)
    axes = vals[5:]
    if data_type == 16:
        # X/Y/Z in µm → mm,  Rx/Ry/Rz in 0.0001° → °
        for i, name in enumerate(["X", "Y", "Z", "Rx", "Ry", "Rz"]):
            if i < len(axes):
                out[f"pos_{name}"] = (
                    round(axes[i] / 1000.0,  3) if i < 3 else
                    round(axes[i] / 10000.0, 4)
                )
    else:
        pulse_names = names or ("S", "L", "U", "R", "B", "T")
        for i, name in enumerate(pulse_names):
            if i < len(axes):
                out[f"pos_{name}"] = axes[i]
    return out


# ── quick self-test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    host = sys.argv[1] if len(sys.argv) > 1 else "192.0.2.10"
    print(f"Testing HSES at {host}:{HSES_PORT}")

    with HsesClient(host, timeout=3.0) as c:
        try:
            s = c.read_status()
            print(f"STATUS:  {s}")
        except Exception as e:
            print(f"STATUS:  ERROR — {e}")

        try:
            a = c.read_alarm(1)
            print(f"ALARM 1: {a}")
        except Exception as e:
            print(f"ALARM 1: ERROR — {e}")

        try:
            j = c.read_job(1)
            print(f"JOB:     {j}")
        except Exception as e:
            print(f"JOB:     ERROR — {e}")

        try:
            p = c.read_position(1)
            print(f"POS:     {p}")
        except Exception as e:
            print(f"POS:     ERROR — {e}")
