"""
RobotConnection — facade bundling both robot transports behind one IP.

  - HSES (UDP 10040)          : High Speed Ethernet Server — fast polling,
    status, plaintext alarms, IO, variables. The preferred path.
  - Ethernet Server (TCP 80)  : classic host-control path (servo, jobs, IO write,
    SETMJ ...), wrapped as a thread-safe, self-reconnecting persistent session.

Which path is the *active* one is decided by `transport` ("hses" | "ethernet").
There is **no** automatic fallback: the chosen path is used. Both clients stay
reachable at all times (`.hses` / `.tcp`), so the few TCP-only commands still
work in HSES mode and, conversely, a controller without the Ethernet Server
option enabled remains usable for telemetry.

Deliberately slim: no FTP backup, no file control, no recording/poll loop and
no per-model axis names — specialised concerns that belong in applications,
not in the communication core.
"""
from __future__ import annotations

import inspect
import logging
import threading
from concurrent.futures import ThreadPoolExecutor

from .hses_client import HsesClient, MODEL_YRC1000
from .yaskawa_client import YaskawaClient

log = logging.getLogger(__name__)

HSES_PORT = 10040
TCP_PORT = 80

# Accepted values for the transport setting.
TRANSPORT_HSES = "hses"
TRANSPORT_ETHERNET = "ethernet"
VALID_TRANSPORTS = (TRANSPORT_HSES, TRANSPORT_ETHERNET)

# VarType value (int) -> HSES client method
_VARTYPE_HSES_READ = {0: "read_b_var", 1: "read_i_var", 2: "read_d_var",
                      3: "read_r_var", 7: "read_s_var"}
_VARTYPE_HSES_WRITE = {0: "write_b_var", 1: "write_i_var", 2: "write_d_var",
                       3: "write_r_var", 7: "write_s_var"}

from .yaskawa_client import VarType as _VT
_POSVAR_ETH = {"P": _VT.ROBOT_POS, "BP": _VT.BASE_POS, "EX": _VT.STATION_POS}


class HsesWriteBlockedError(RuntimeError):
    """Switching an IO output in HSES mode — deliberately blocked.

    `HsesClient.write_io()` (yrcom/hses_client.py) always writes through
    `write_io_plural()`, which requires an even byte count. A single byte is
    padded with a hard `0x00` — so the byte following the target byte is
    zeroed regardless of its actual state. Switching OT#4 (instance 1001)
    would silently clear instance 1002 (OT#9–OT#16) — real actuators in the
    robot cell. Since it is additionally unverified whether the controller
    accepts output writes over this path at all, the write path stays blocked
    instead of compensating the neighbour-byte effect on speculation.
    `hses_client.py` is deliberately left unchanged. The read path
    (`read_io_bytes`) is not affected by this defect and stays untouched.
    """


class PersistentTcpClient:
    """
    Thread-safe, self-reconnecting TCP Ethernet Server session (keep-alive).

    Forwards every attribute access to the underlying YaskawaClient; on a
    broken socket it reconnects once and retries the call. Passes
    `persistent=True` where the method supports it.
    """

    def __init__(self, ip: str, port: int = TCP_PORT, timeout: float = 5.0) -> None:
        self._ip = ip
        self._port = port
        self._timeout = timeout
        self._client: YaskawaClient | None = None
        self._lock = threading.Lock()
        self._connected = False

    def _connect(self) -> None:
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        client = YaskawaClient(self._ip, self._port, timeout=self._timeout)
        client.connect()
        client.send_start(keep_alive=-1)  # unlimited keep-alive session
        self._client = client
        self._connected = True

    def close(self) -> None:
        self._connected = False
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    def __getattr__(self, name: str):
        def wrapper(*args, **kwargs):
            with self._lock:
                for attempt in range(2):
                    try:
                        if not self._connected or self._client is None:
                            self._connect()
                        method = getattr(self._client, name)
                        try:
                            sig = inspect.signature(method)
                            if "persistent" in sig.parameters and "persistent" not in kwargs:
                                kwargs["persistent"] = True
                        except (ValueError, TypeError):
                            pass
                        return method(*args, **kwargs)
                    except (OSError, ConnectionError, BrokenPipeError):
                        if attempt == 0:
                            self.close()
                            continue
                        raise

        return wrapper


class RobotConnection:
    """One logical connection to the robot, bundling HSES (UDP) + Ethernet Server (TCP)."""

    def __init__(
        self,
        ip: str,
        hses_port: int = HSES_PORT,
        tcp_port: int = TCP_PORT,
        timeout: float = 2.0,
        bind_host: str | None = None,
        transport: str = TRANSPORT_HSES,
    ) -> None:
        if transport not in VALID_TRANSPORTS:
            raise ValueError(
                f"Unknown transport {transport!r} — allowed: {VALID_TRANSPORTS}"
            )
        self.ip = ip
        self.hses_port = hses_port
        self.tcp_port = tcp_port
        self.timeout = timeout
        self.bind_host = bind_host   # local adapter IP for the HSES socket (optional)
        self.transport = transport   # "hses" (UDP) or "ethernet" (TCP host control)
        self._hses: HsesClient | None = None
        self._tcp: PersistentTcpClient | None = None
        self._connected = False
        self._lock = threading.Lock()

    @classmethod
    def from_settings(cls, cfg: dict) -> "RobotConnection":
        """Build a connection from a settings dict (robot_ip, robot_port, hses_port, transport)."""
        return cls(
            ip=cfg["robot_ip"],
            tcp_port=cfg.get("robot_port", TCP_PORT),
            hses_port=cfg.get("hses_port", HSES_PORT),
            transport=cfg.get("transport", TRANSPORT_HSES),
        )

    # ── lifecycle ─────────────────────────────────────────────────────────────
    def connect(self) -> dict:
        """Open the chosen transport and verify reachability with a status read."""
        with self._lock:
            if self.transport == TRANSPORT_ETHERNET:
                tcp = PersistentTcpClient(self.ip, self.tcp_port, max(self.timeout, 5.0))
                status = tcp.read_rstats(decode=True)  # raises when unreachable
                self._tcp = tcp
            else:
                hses = HsesClient(self.ip, self.hses_port, self.timeout,
                                  bind_host=self.bind_host, model=MODEL_YRC1000)
                hses.open()
                try:
                    status = hses.read_status()  # raises when unreachable / bad reply
                    try:
                        hses.detect_model()       # determine generation (status bit 4), best effort
                    except Exception:
                        pass
                except Exception:
                    hses.close()
                    raise
                self._hses = hses
            self._connected = True
        return status

    def disconnect(self) -> None:
        with self._lock:
            if self._hses:
                self._hses.close()
                self._hses = None
            if self._tcp:
                self._tcp.close()
                self._tcp = None
            self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    # ── high-speed path (HSES) ────────────────────────────────────────────────
    @property
    def hses(self) -> HsesClient:
        """Lazily created HSES client (UDP), usable regardless of the active transport."""
        if not self._hses:
            self._hses = HsesClient(self.ip, self.hses_port, self.timeout,
                                    bind_host=self.bind_host, model=MODEL_YRC1000)
            self._hses.open()
        return self._hses

    # ── classic path (TCP Ethernet Server) ────────────────────────────────────
    @property
    def tcp(self) -> PersistentTcpClient:
        """Lazily created persistent TCP host-control session."""
        if not self._tcp:
            self._tcp = PersistentTcpClient(self.ip, self.tcp_port, max(self.timeout, 5.0))
        return self._tcp

    # ── shared query ──────────────────────────────────────────────────────────
    def status(self) -> dict:
        """Fast live status read over the active transport.

        Both paths return the same set of status keys (RSTATS ≡ read_status) —
        callers cannot tell which path is currently active.
        """
        with self._lock:
            if self.transport == TRANSPORT_ETHERNET:
                return self.tcp.read_rstats(decode=True)
            return self.hses.read_status()

    # ── unified read interface (dispatch by transport) ────────────────────────
    def _use_hses(self) -> bool:
        return self.transport == TRANSPORT_HSES

    def capabilities(self) -> set[str]:
        """Which data sources the *active* transport can read.

        Registers (M) can only be read over HSES; the text-based Ethernet
        Server has no register read command. Derived purely from the
        transport — no connection required.
        """
        if self._use_hses():
            return {"var", "io", "status", "register"}
        return {"var", "io", "status"}

    def read_rstats(self) -> dict:
        if self._use_hses():
            return self.hses.read_status()
        return self.tcp.read_rstats(decode=True)

    def read_variable(self, var_type, var_no):
        if self._use_hses():
            name = _VARTYPE_HSES_READ.get(int(var_type))
            if name is None:
                raise ValueError(f"HSES read: VarType {var_type!r} not supported")
            return getattr(self.hses, name)(var_no)
        return self.tcp.read_variable(var_type, var_no)

    def read_variable_block(self, var_tuples: list[tuple]) -> dict:
        """Read several variables; returns {(VarType, var_no): value}.

        Single reads in a loop over the active transport. The TCP path runs
        them as persistent single commands on the open long-lived session
        (`PersistentTcpClient` passes `persistent=True`); the batched block
        commands of the YaskawaClient class open a session of their own and
        do not get along with the persistent connection.
        """
        out: dict = {}
        for var_type, var_no in var_tuples:
            out[(var_type, var_no)] = self.read_variable(var_type, var_no)
        return out

    def read_register(self, number: int) -> int:
        """Read an M register (HSES only). The ES has no register read command."""
        if self._use_hses():
            vals = self.hses.read_registers(number, 1)
            return vals[0] if vals else 0
        raise NotImplementedError("Reading registers is only possible over HSES")

    def read_position_var(self, vartype: str, number: int) -> list:
        """Read a position variable P/BP/EX; returns the axis values as a list."""
        if self._use_hses():
            if vartype == "P":
                return list(self.hses.read_position_var(number).axes)
            if vartype == "BP":
                return list(self.hses.read_bp_var(number).axes)
            if vartype == "EX":
                return list(self.hses.read_ex_var(number).axes)
            raise ValueError(f"Not a position type: {vartype!r}")
        vt = _POSVAR_ETH.get(vartype)
        if vt is None:
            raise ValueError(f"Not a position type: {vartype!r}")
        val = self.tcp.read_variable(vt, number)
        return list(val) if isinstance(val, (list, tuple)) else [val]

    def read_io_bit(self, address: int) -> bool:
        if self._use_hses():
            byte = self.hses.read_io(address)   # read_io divides by 10 internally
            return bool((byte >> (address % 10)) & 1)
        return self.tcp.read_io_bit(address)

    def read_io_bytes(self, start_logical: int, count: int) -> list[int]:
        """Read several IO bytes in one go (for batched polling).

        start_logical is the full Yaskawa address (e.g. 50050); one byte
        spans 10 address units.
        """
        if self._use_hses():
            # read_io_plural requires an even count — round up, then trim.
            want = count + (count % 2)
            data = self.hses.read_io_plural(start_logical, want)
            return list(data)[:count]
        # The ES counts in bits.
        return self.tcp.io_read(start_logical, count * 8)

    def write_io_byte(self, logical: int, value: int) -> bool:
        """Write one IO byte — Ethernet Server only.

        Blocked in HSES mode: see `HsesWriteBlockedError`. The Ethernet
        branch passes `tcp.io_write()` through unchanged (including `False`)
        instead of swallowing it — a swallowed failure would report a switch
        operation to the operator that never happened.
        """
        if self._use_hses():
            raise HsesWriteBlockedError(
                "Switching outputs is blocked in HSES mode: "
                "HsesClient.write_io() pads a single-byte write to an even "
                "byte count internally, forcing the following byte to 0x00 — "
                "a switch operation would silently clear up to eight "
                "unrelated outputs (see yrcom/hses_client.py, write_io). "
                "Remedy: use transport=\"ethernet\" to switch outputs."
            )
        return bool(self.tcp.io_write(logical, [value]))

    def read_job_seq(self) -> dict:
        if self._use_hses():
            j = self.hses.read_job()
            return {"JobName": j.get("job_name"),
                    "Line": j.get("job_line"),
                    "Step": j.get("job_step")}
        return self.tcp.read_job_seq()

    def read_alarm(self):
        """HSES: plaintext dict (alarm_code/alarm_name/alarm_time/...).
        ES: raw code list (RALARM). Callers handle both shapes."""
        if self._use_hses():
            return self.hses.read_alarm(1)
        return self.tcp.read_alarm()

    # ── unified control interface ─────────────────────────────────────────────
    def write_variable(self, var_type, var_no, value) -> bool:
        if self._use_hses():
            name = _VARTYPE_HSES_WRITE.get(int(var_type))
            if name is None:
                raise ValueError(f"HSES write: VarType {var_type!r} not supported")
            getattr(self.hses, name)(var_no, value)
            return True
        return self.tcp.write_variable(var_type, var_no, value)

    def write_variable_block(self, write_tuples: list[tuple]) -> bool:
        """Write several variables; tuples are (VarType, var_no, value).

        Single writes in a loop over the active transport; True only if all
        of them succeed. The TCP path runs as persistent single commands on
        the long-lived session (not the batched block command, which would
        open a session of its own). Unlike the IO byte write, the variable
        write over HSES is not affected by the neighbour-byte defect
        (dedicated per-type commands), so it stays allowed. Callers verify
        by reading back anyway.
        """
        ok = True
        for var_type, var_no, value in write_tuples:
            ok = self.write_variable(var_type, var_no, value) and ok
        return ok

    def servo_on(self, on: bool = True) -> bool:
        if self._use_hses():
            self.hses.servo(on)
            return True
        return self.tcp.servo_on(on)

    def hold(self, on: bool = True) -> bool:
        if self._use_hses():
            self.hses.hold(on)
            return True
        return self.tcp.hold(on)

    def start_job(self, jobname=None) -> bool:
        if self._use_hses():
            self.hses.start_job()   # HSES starts the selected job (no name)
            return True
        return self.tcp.start_job(jobname)

    def set_job_seq(self, jobname, line) -> bool:
        if self._use_hses():
            self.hses.select_job(jobname, line)
            return True
        return self.tcp.set_job_seq(jobname, line)

    # ── reachability probe ────────────────────────────────────────────────────
    @staticmethod
    def _probe(action) -> bool:
        try:
            action()
            return True
        except Exception:
            return False

    def _probe_hses(self) -> bool:
        def attempt():
            c = HsesClient(self.ip, self.hses_port, 2.0, bind_host=self.bind_host)
            c.open()
            try:
                c.read_status()
            finally:
                c.close()

        return self._probe(attempt)

    def _probe_ethernet(self) -> bool:
        return self._probe(lambda: YaskawaClient(self.ip, self.tcp_port, timeout=2.0).read_rstats())

    def detect(self) -> dict:
        """Probe both transports concurrently; report which one answers.

        Useful during setup: whether a cell has only one, only the other or
        both servers enabled is not known in advance.
        """
        with ThreadPoolExecutor(max_workers=2) as ex:
            hses = ex.submit(self._probe_hses)
            eth = ex.submit(self._probe_ethernet)
            return {"hses": hses.result(), "ethernet": eth.result()}
