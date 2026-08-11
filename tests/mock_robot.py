"""
Mock Yaskawa Ethernet Server for integration testing.

Implements a minimal-but-correct subset of the HOSTCTRL_REQUEST protocol so the
real `YaskawaClient` SDK can talk to it without a physical robot. State is held
in-memory and can be inspected from the test side.

Wire protocol (per command transaction):
    1. CONNECT Robot_access [Keep-Alive:N]\r\n   →  OK: NX Information Server\r\n
    2. HOSTCTRL_REQUEST {CMD} {size}\r\n         →  OK: {CMD}\r\n
    3. (optional) {data}\r                       →  {answer}\r
    4. Repeat 2-3 according to Keep-Alive setting, then close.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Any


# ── Bit-field encoding for RSTATS ────────────────────────────────────────────

def _encode_rstats(
    *,
    running: bool,
    play_mode: bool,
    cmd_remote: bool,
    hold_active: bool,
    alarm: bool,
    error: bool,
    servo_on: bool,
) -> tuple[int, int]:
    """
    Real robot calibration data (192.168.1.24):
      Teach+ServoOFF  →  34,0   data1=0b00100010  (bits 1+5 always set in teach)
      Teach+ServoON   →  34,64  data2=0b01000000  (bit 6 = servo_on)
      Play+ServoOFF   →  68,0   data1=0b01000100  (bits 2+6, bit6=play)
      Play+ServoON    →  68,64
      Remote+ServoON  →  194,64 data1=0b11000010  (bits 1+6+7)

    data1:  D7=cmd_remote  D6=play  D3=running
            D5=teach_mode_flag (set when not in play/remote)
            D2=play_mode_flag  (set when in play, not in remote)
            D1=always set in play/remote
    data2:  D6=servo_on  D5=error  D4=alarm  D3=hold_cmd
    """
    d1 = 0
    if running:    d1 |= 1 << 3
    if play_mode:  d1 |= 1 << 6
    if cmd_remote: d1 |= 1 << 7

    # Extra bits observed on real robot
    if not play_mode and not cmd_remote:
        d1 |= (1 << 5) | (1 << 1)   # teach mode: bits 5+1
    elif play_mode and not cmd_remote:
        d1 |= 1 << 2                  # play mode: bit 2
    elif cmd_remote:
        d1 |= 1 << 1                  # remote mode: bit 1

    d2 = 0
    if hold_active: d2 |= 1 << 3
    if alarm:       d2 |= 1 << 4
    if error:       d2 |= 1 << 5
    if servo_on:    d2 |= 1 << 6
    return d1, d2


# ── Mock server ──────────────────────────────────────────────────────────────

class MockRobotServer:
    """
    In-memory Yaskawa controller. Listens on 127.0.0.1, OS-picked port.

    Usage:
        with MockRobotServer() as srv:
            srv.servo_on = True
            srv.set_io_bit(10010, True)
            # ... point a YaskawaClient at srv.port ...
    """

    def __init__(self, host: str = "127.0.0.1") -> None:
        self.host = host
        self.port: int = 0
        self._sock: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._client_threads: list[threading.Thread] = []

        # Robot state (public; tests read/write directly)
        self.servo_on: bool = False
        self.running: bool = False
        self.hold_active: bool = False
        self.alarm: bool = False
        self.error: bool = False
        self.cmd_remote: bool = True
        self.play_mode: bool = True
        self.variables: dict[tuple[int, int], Any] = {}
        # Safety fence (80023) active by default — mirrors real robot at 192.168.1.24
        self.io_bytes: dict[int, int] = {80020: 0b00000100}
        self.master_job: str = "SLEEP"
        self.current_job: str = "SLEEP"
        self.jobs: list[str] = [
            "DIST_FB_POS_SPHERE", "CHECK_ROOMS", "WARTUNGTEST",
            "DIST_FB_POS_CYLINDER", "3D_DISTWINKEL", "STUNDEN",
            "SETUP-SAND", "RADIUS_POS", "SANDUHRLOGO_HAL",
            "HOME", "SETUP", "WARTUNGTESTMOVE", "SLEEP",
        ]

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def __enter__(self) -> "MockRobotServer":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, 0))
        self._sock.listen(16)
        self._sock.settimeout(0.5)  # so accept loop can poll _stop_event
        self.port = self._sock.getsockname()[1]
        self._stop_event.clear()
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True, name="mock-robot-accept"
        )
        self._accept_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)
            self._accept_thread = None

    # ── State management ────────────────────────────────────────────────────

    def reset_state(self) -> None:
        """Reset all robot state to defaults — call between tests."""
        self.servo_on = False
        self.running = False
        self.hold_active = False
        self.alarm = False
        self.error = False
        self.cmd_remote = True
        self.play_mode = True
        self.variables.clear()
        # Safety fence IO bit 80023 = True (base 80020, bit 3 = 0b00000100)
        self.io_bytes = {80020: 0b00000100}
        self.master_job = "SLEEP"
        self.current_job = "SLEEP"
        self.jobs = [
            "DIST_FB_POS_SPHERE", "CHECK_ROOMS", "WARTUNGTEST",
            "DIST_FB_POS_CYLINDER", "3D_DISTWINKEL", "STUNDEN",
            "SETUP-SAND", "RADIUS_POS", "SANDUHRLOGO_HAL",
            "HOME", "SETUP", "WARTUNGTESTMOVE", "SLEEP",
        ]

    def set_io_bit(self, rob_addr: int, state: bool) -> None:
        bit = rob_addr % 10
        base = rob_addr - bit
        cur = self.io_bytes.get(base, 0)
        self.io_bytes[base] = (cur | (1 << bit)) if state else (cur & ~(1 << bit))

    def get_io_bit(self, rob_addr: int) -> bool:
        bit = rob_addr % 10
        base = rob_addr - bit
        return bool((self.io_bytes.get(base, 0) >> bit) & 1)

    def set_variable(self, vartype: int, var_no: int, value: Any) -> None:
        self.variables[(int(vartype), var_no)] = value

    def get_variable(self, vartype: int, var_no: int, default: Any = 0) -> Any:
        return self.variables.get((int(vartype), var_no), default)

    # ── Accept loop ─────────────────────────────────────────────────────────

    def _accept_loop(self) -> None:
        assert self._sock is not None
        while not self._stop_event.is_set():
            try:
                conn, _addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed
            t = threading.Thread(
                target=self._handle_client, args=(conn,), daemon=True,
                name="mock-robot-conn",
            )
            t.start()
            self._client_threads.append(t)

    def _handle_client(self, conn: socket.socket) -> None:
        try:
            conn.settimeout(2.0)
            # 1. CONNECT line
            connect_line = self._read_until(conn, b"\r\n")
            if not connect_line:
                return
            keep_alive = self._parse_keep_alive(connect_line)
            conn.sendall(b"OK: NX Information Server\r\n")

            # 2. Loop over HOSTCTRL_REQUEST commands.
            # Keep-Alive: None=close after 1 cmd, -1=infinite, N=N commands.
            # We do NOT proactively close: instead we keep the socket open and
            # let the client close it (or EOF/timeout will end this loop).
            # This avoids a race where the mock's close happens before the
            # client has consumed the response.
            commands_remaining = keep_alive
            count = 0
            while True:
                if commands_remaining is not None and commands_remaining != -1:
                    if count >= commands_remaining:
                        # Wait for client to close — read returns b'' on EOF.
                        try:
                            conn.settimeout(2.0)
                            conn.recv(1)
                        except (socket.timeout, OSError):
                            pass
                        break

                header = self._read_until(conn, b"\r\n")
                if not header:
                    break
                cmd, size = self._parse_header(header)
                if cmd is None:
                    break

                conn.sendall(f"OK: {cmd}\r\n".encode("ascii"))

                data = ""
                if size > 0:
                    data_bytes = self._read_until(conn, b"\r")
                    data = data_bytes.decode("ascii", errors="replace").rstrip("\r")

                response = self._dispatch(cmd, data)
                if not response.endswith("\r"):
                    response += "\r"
                # Tiny delay so the OK and answer land in separate recv() calls
                # on the client side. Real robots do this naturally via network
                # latency; without it the SDK's _recv_until_ok over-reads the
                # answer and _recv_answer blocks on an empty socket.
                time.sleep(0.005)
                conn.sendall(response.encode("ascii"))
                count += 1
        except (socket.timeout, OSError, ConnectionResetError, BrokenPipeError):
            pass
        except Exception:
            # Test mock: never crash the server thread on a parse error
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    # ── Wire helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _read_until(conn: socket.socket, terminator: bytes) -> bytes:
        """Byte-by-byte read until terminator appears. Returns b'' on EOF."""
        buf = bytearray()
        while terminator not in buf:
            try:
                chunk = conn.recv(1)
            except (socket.timeout, OSError):
                return b""
            if not chunk:
                return b""
            buf.extend(chunk)
        return bytes(buf)

    @staticmethod
    def _parse_keep_alive(line: bytes) -> int | None:
        """CONNECT Robot_access [Keep-Alive:N]\r\n → N or None."""
        text = line.decode("ascii", errors="replace").strip()
        if "Keep-Alive:" not in text:
            return None
        try:
            tail = text.split("Keep-Alive:", 1)[1].strip()
            return int(tail)
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _parse_header(line: bytes) -> tuple[str | None, int]:
        """HOSTCTRL_REQUEST {CMD} {size}\r\n → (CMD, size)."""
        text = line.decode("ascii", errors="replace").strip()
        parts = text.split()
        if len(parts) < 3 or parts[0] != "HOSTCTRL_REQUEST":
            return None, 0
        try:
            return parts[1], int(parts[2])
        except ValueError:
            return parts[1], 0

    # ── Command dispatch ────────────────────────────────────────────────────

    def _dispatch(self, cmd: str, data: str) -> str:
        handler_name = f"_cmd_{cmd.lower()}"
        handler = getattr(self, handler_name, None)
        if handler is None:
            return "1011"  # unknown / bad parameter format
        try:
            return handler(data)
        except Exception:
            return "1011"

    # ── Status ─────────────────────────────────────────────────────────────

    def _cmd_rstats(self, _data: str) -> str:
        d1, d2 = _encode_rstats(
            running=self.running,
            play_mode=self.play_mode,
            cmd_remote=self.cmd_remote,
            hold_active=self.hold_active,
            alarm=self.alarm,
            error=self.error,
            servo_on=self.servo_on,
        )
        return f"{d1},{d2}"

    def _cmd_ralarm(self, _data: str) -> str:
        if self.alarm:
            return "4315,0,0,0,0,0,0,0,0,0"
        return "0,0,0,0,0,0,0,0,0,0"

    def _cmd_rposj(self, _data: str) -> str:
        return "0,0,0,0,0,0"

    def _cmd_rposc(self, _data: str) -> str:
        return "0.0,0.0,0.0,0.0,0.0,0.0,0,0"

    def _cmd_rjseq(self, _data: str) -> str:
        return f"{self.current_job},0,0"

    def _cmd_rjdir(self, _data: str) -> str:
        return ",".join(self.jobs)

    # ── Variables ──────────────────────────────────────────────────────────

    def _cmd_savev(self, data: str) -> str:
        # data: "{vt},{vno}"
        parts = [p for p in data.split(",") if p != ""]
        vt = int(parts[0])
        vno = int(parts[1])
        value = self.variables.get((vt, vno), 0)
        if isinstance(value, (list, tuple)):
            return ",".join(str(v) for v in value)
        return str(value)

    def _cmd_loadv(self, data: str) -> str:
        # data: "{vt},{vno},{value...}"
        parts = data.split(",")
        if len(parts) < 3:
            return "1011"
        vt = int(parts[0])
        vno = int(parts[1])
        rest = parts[2:]
        value: Any
        if len(rest) == 1:
            raw = rest[0]
            try:
                value = int(raw)
            except ValueError:
                try:
                    value = float(raw)
                except ValueError:
                    value = raw
        else:
            value = [self._coerce_num(p) for p in rest]
        self.variables[(vt, vno)] = value
        return "0000"

    @staticmethod
    def _coerce_num(s: str) -> Any:
        try:
            return int(s)
        except ValueError:
            try:
                return float(s)
            except ValueError:
                return s

    # ── IO ─────────────────────────────────────────────────────────────────

    def _cmd_ioread(self, data: str) -> str:
        # data: "{start},{num_points}"
        parts = [p for p in data.split(",") if p != ""]
        start = int(parts[0])
        num_points = int(parts[1])
        num_bytes = num_points // 8
        out = []
        for i in range(num_bytes):
            base = start + i * 10
            out.append(str(self.io_bytes.get(base, 0)))
        return ",".join(out)

    def _cmd_iowrite(self, data: str) -> str:
        # data: "{start},{num_points},{b1},{b2},..."
        parts = [p for p in data.split(",") if p != ""]
        if len(parts) < 3:
            return "1011"
        start = int(parts[0])
        num_points = int(parts[1])
        num_bytes = num_points // 8
        byte_vals = [int(p) & 0xFF for p in parts[2:2 + num_bytes]]
        for i, bv in enumerate(byte_vals):
            base = start + i * 10
            self.io_bytes[base] = bv
        return "0000"

    # ── Control ────────────────────────────────────────────────────────────

    def _cmd_svon(self, data: str) -> str:
        self.servo_on = data.strip() == "1"
        # NW bit 50073 = servo-on indicator (real robot sets this automatically)
        cur = self.io_bytes.get(50070, 0)
        if self.servo_on:
            self.io_bytes[50070] = cur | (1 << 3)
        else:
            self.io_bytes[50070] = cur & ~(1 << 3)
        return "0000"

    def _cmd_hold(self, data: str) -> str:
        self.hold_active = data.strip() == "1"
        return "0000"

    def _cmd_reset(self, _data: str) -> str:
        self.alarm = False
        self.error = False
        return "0000"

    def _cmd_cancel(self, _data: str) -> str:
        self.error = False
        return "0000"

    def _cmd_start(self, data: str) -> str:
        jobname = data.strip()
        if jobname:
            if not self.servo_on:
                return "2070"
            self.current_job = jobname
        self.running = True
        return "0000"

    def _cmd_setmj(self, data: str) -> str:
        name = data.strip()
        if not name:
            return "1011"
        self.master_job = name
        return "0000"

    def _cmd_getmj(self, _data: str) -> str:
        return self.master_job
