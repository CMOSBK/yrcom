"""
Yaskawa Ethernet Server client — standalone, zero external dependencies.

Supports NX100 and YRC1000 controllers via the HOSTCTRL_REQUEST protocol
(Ethernet Server Function, TCP port 80).

Protocol references:
  NX100:   HW0483202  (NX100 OPTIONS INSTRUCTIONS FOR ETHERNET SERVER FUNCTION)
  YRC1000: HW1483358  Chapter 8  (Ethernet Server Function — backward compatible)

Typical usage
─────────────
# Single transaction (auto connect/close):
    client = YaskawaClient("192.168.1.24")
    status = client.read_rstats()

# Persistent connection (context manager):
    with YaskawaClient("192.168.1.24", timeout=2) as client:
        client.send_start(keep_alive=3)
        v = client.read_variable(VarType.BYTE, 10, persistent=True)
        client.write_variable(VarType.BYTE, 10, 5, persistent=True)

# Block write (one TCP session, N commands):
    with YaskawaClient(ip) as client:
        ok = client.write_variable_block([
            (VarType.BYTE, 10, 5),
            (VarType.DOUBLE, 1, 1000),
        ])
"""

from __future__ import annotations

import socket
from typing import Any, Literal, overload

from enum import IntEnum


class VarType(IntEnum):
    """Yaskawa variable types (SAVEV / LOADV Data-1 parameter).

    Single source of truth for yrcom.
    NX100 manual §3.2 / YRC1000 manual Ch.8 §8.3.2.
    """

    BYTE        = 0  # B-variable (8-bit)
    INTEGER     = 1  # I-variable (signed 16-bit)
    DOUBLE      = 2  # D-variable (double-precision INTEGER, signed 32-bit — not a float!)
    FLOAT       = 3  # R-variable (real / IEEE-754 float)
    ROBOT_POS   = 4  # P-variable (robot-axis position)
    BASE_POS    = 5  # BP-variable (base-axis position)
    STATION_POS = 6  # EX-variable / station-axis position (pulse type only)
    STRING      = 7  # S-variable (character string)

# ── Public constants ──────────────────────────────────────────────────────────

DEFAULT_PORT: int = 80
DEFAULT_TIMEOUT: float = 5.0
_RECV_SIZE: int = 4096


# ── Type aliases ──────────────────────────────────────────────────────────────

ParsedData = list[int] | list[str] | dict[str, Any]


# ── Client ────────────────────────────────────────────────────────────────────

class YaskawaClient:
    """
    Yaskawa Ethernet Server protocol client (NX100 / YRC1000).

    Protocol flow
    ─────────────
    1. TCP connect to robot_ip:80
    2. Send:  CONNECT Robot_access\\r\\n
              (or CONNECT Robot_access Keep-Alive:N\\r\\n for N commands)
    3. Recv:  OK: NX Information Server ...\\r\\n
    4. Send:  HOSTCTRL_REQUEST {CMD} {len(data)}\\r\\n
    5. Recv:  OK: {CMD}\\r\\n
    6. Send:  {data}\\r              (if len(data) > 0)
    7. Recv:  {answer}\\r            (or {answer}\\r\\n for "0000" acks)
    8. Repeat steps 4-7 if Keep-Alive; socket auto-closes after last command.

    Notes
    ─────
    - Data size in the HOSTCTRL_REQUEST header is the byte length of the data
      string *including* the trailing \\r.
    - IO data is always in multiples of 8 contact points (one byte per 8 bits).
    - IOWRITE can only target network inputs #22010–#23287 on NX100.
    - START: pass job name without the .JBI extension.
    - SETMJ/GETMJ require Remote/Play mode on the controller.
    - write + read-back: always use separate send_start() sessions to avoid
      double send_start() on the same connection returning empty responses.
    """

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None
        # Bytes already received but not yet consumed. TCP is a byte stream: one recv() may
        # hold a header and its answer together, or only part of a line.
        self._rx = b""

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self) -> "YaskawaClient":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open TCP socket to the controller. No-op if already connected."""
        if self.sock is not None:
            return
        self._rx = b""
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except Exception:
            self.sock = None
            raise

    def close(self) -> None:
        """Close TCP socket."""
        if self.sock:
            self.sock.close()
            self.sock = None
        self._rx = b""

    # ── Protocol primitives ───────────────────────────────────────────────────

    def send_start(self, keep_alive: int | None = None) -> str:
        """
        Send CONNECT Robot_access handshake.

        keep_alive=None  → single-command session (socket closes after 1 command)
        keep_alive=N     → multi-command session (N commands before close)
        keep_alive=-1    → infinite keep-alive (close manually or via context mgr)
        """
        if self.sock is None:
            raise RuntimeError("Socket not connected — call connect() first")
        if keep_alive is None:
            line = "CONNECT Robot_access\r\n"
        else:
            line = f"CONNECT Robot_access Keep-Alive:{keep_alive}\r\n"
        self.sock.sendall(line.encode("ascii"))
        return self._recv_until_ok()

    def send_command(self, command: str, size: int = 0, data: str = "") -> str:
        """
        Send HOSTCTRL_REQUEST header and optional data, return raw answer string.
        Expects send_start() to have been called on this session already.
        """
        if self.sock is None:
            raise RuntimeError("Socket not connected")
        header = f"HOSTCTRL_REQUEST {command} {size}\r\n"
        self.sock.sendall(header.encode("ascii"))
        reply = self._recv_until_ok()
        if reply.startswith("NG"):
            # Request refused: no answer line follows, so waiting for one would only time out.
            return reply
        if size > 0 and data:
            payload = data if data.endswith("\r") else data + "\r"
            self.sock.sendall(payload.encode("ascii"))
        return self._recv_answer()

    def _recv_line(self) -> str | None:
        """Return the next line from the stream without its terminator, or None
        if the peer closed the connection with nothing left to read.

        Lines end in ``\r\n`` (handshake and header replies) or a bare ``\r``
        (answer data). Reads until a terminator is buffered, so a reply split
        over several segments, or several replies in one segment, are handled
        the same way. A ``\n`` left over from ``\r\n`` split across segments is
        skipped at the start of the next line.
        """
        if self.sock is None:
            raise RuntimeError("Socket not connected")
        while True:
            self._rx = self._rx.lstrip(b"\n")
            end = self._rx.find(b"\r")
            if end >= 0:
                line, self._rx = self._rx[:end], self._rx[end + 1:]
                if self._rx.startswith(b"\n"):
                    self._rx = self._rx[1:]
                return line.decode("ascii")
            chunk = self.sock.recv(_RECV_SIZE)
            if not chunk:
                # Peer closed: hand out an unterminated rest once, then report the end.
                line, self._rx = self._rx, b""
                return line.decode("ascii") if line else None
            self._rx += chunk

    def _recv_until_ok(self) -> str:
        """Receive lines until one starts with OK or NG and return it (with ``\r\n``);
        "" if the peer closed first."""
        while True:
            line = self._recv_line()
            if line is None:
                return ""
            if line.startswith(("OK", "NG")):
                return line + "\r\n"

    def _recv_answer(self) -> str:
        """Receive one answer line (data ends with ``\r`` or ``\r\n``); "" if the peer closed."""
        line = self._recv_line()
        return "" if line is None else line + "\r"

    # ── Transaction patterns ──────────────────────────────────────────────────

    @overload
    def transaction(self, command: str, size: int = ..., data: str = ...,
                    keep_alive: int | None = ..., parse: Literal[True] = ...) -> ParsedData: ...
    @overload
    def transaction(self, command: str, size: int = ..., data: str = ...,
                    keep_alive: int | None = ..., parse: Literal[False] = ...) -> str: ...

    def transaction(
        self,
        command: str,
        size: int = 0,
        data: str = "",
        keep_alive: int | None = None,
        parse: bool = True,
    ) -> ParsedData | str:
        """
        Self-contained transaction: auto-connect if needed, auto-close if we opened.
        When called inside an open with-block, reuses the existing socket.
        """
        was_connected = self.sock is not None
        self.connect()
        try:
            self.send_start(keep_alive)
            raw = self.send_command(command, size, data)
            if parse:
                part = raw.split("OK")[-1] if "OK" in raw else raw
                return self._parse(command, part.strip())
            return raw
        finally:
            if not was_connected:
                self.close()

    @overload
    def send_transaction(self, command: str, size: int = ..., data: str = ...,
                         parse: Literal[True] = ...) -> ParsedData: ...
    @overload
    def send_transaction(self, command: str, size: int = ..., data: str = ...,
                         parse: Literal[False] = ...) -> str: ...

    def send_transaction(
        self,
        command: str,
        size: int = 0,
        data: str = "",
        parse: bool = True,
    ) -> ParsedData | str:
        """
        Execute one command on an *already-open* session (after send_start()).
        Use this inside a with-block for multi-command sessions.
        """
        raw = self.send_command(command, size, data)
        if parse:
            part = raw.split("OK")[-1] if "OK" in raw else raw
            return self._parse(command, part.strip())
        return raw

    def _exec(self, persistent: bool, command: str, size: int = 0,
              data: str = "", parse: bool = True) -> ParsedData | str:
        """Route one command to the persistent (send_transaction) or one-shot
        (transaction) path — both share the command/size/data/parse signature."""
        if persistent:
            return self.send_transaction(command, size=size, data=data, parse=parse)
        return self.transaction(command, size=size, data=data, parse=parse)

    # ── Parsing ───────────────────────────────────────────────────────────────

    @staticmethod
    def _clean(text: str) -> str:
        return text.replace("\r\n", "").replace("\r", "").strip()

    def _parse(self, command: str, answer: str) -> ParsedData:
        answer = self._clean(answer)
        parts = [p for p in answer.split(",") if p]

        if command == "RSTATS":
            return [int(p) for p in parts]
        if command == "RPOSJ":
            return [int(p) for p in parts]
        if command == "RPOSC":
            return {
                "X": float(parts[0]), "Y": float(parts[1]), "Z": float(parts[2]),
                "Rx": float(parts[3]), "Ry": float(parts[4]), "Rz": float(parts[5]),
                "Type": int(parts[6]), "ToolNo": int(parts[7]),
            }
        if command == "RJSEQ":
            return {"JobName": parts[0], "Line": int(parts[1]), "Step": int(parts[2])}
        if command == "RALARM":
            return [int(p) for p in parts]
        if command == "IOREAD":
            return [int(p) for p in parts]
        if command == "RJDIR":
            return parts
        return parts

    # ── Variable payload helpers ──────────────────────────────────────────────

    @staticmethod
    def _var_payload(var_type: VarType, var_no: int, value: Any = None) -> str:
        vt = int(var_type)
        if value is None:
            return f"{vt},{var_no}\r"
        if var_type in (VarType.BYTE, VarType.INTEGER, VarType.DOUBLE, VarType.FLOAT, VarType.STRING):
            return f"{vt},{var_no},{value}\r"
        if var_type in (VarType.ROBOT_POS, VarType.BASE_POS, VarType.STATION_POS):
            if isinstance(value, (list, tuple)):
                return f"{vt},{var_no}," + ",".join(str(v) for v in value) + "\r"
            raise ValueError("Position variable types require a list/tuple value")
        raise ValueError(f"Unknown VarType: {var_type}")

    @staticmethod
    def _parse_var(var_type: VarType, raw: str) -> Any:
        if var_type in (VarType.BYTE, VarType.INTEGER, VarType.DOUBLE):
            try:
                return int(raw)
            except (ValueError, TypeError):
                return None
        if var_type == VarType.FLOAT:
            try:
                return float(raw)
            except (ValueError, TypeError):
                return None
        if var_type in (VarType.ROBOT_POS, VarType.BASE_POS, VarType.STATION_POS):
            return [float(p) if "." in p else int(p) for p in raw.split(",") if p]
        return raw  # STRING

    # ── Variable read / write ─────────────────────────────────────────────────

    def read_variable(self, var_type: VarType, var_no: int, persistent: bool = False) -> Any:
        """Read a single variable (SAVEV). persistent=True: call inside existing session."""
        data = self._var_payload(var_type, var_no)
        raw = self._exec(persistent, "SAVEV", size=len(data), data=data, parse=False)
        return self._parse_var(var_type, self._clean(raw))

    def write_variable(self, var_type: VarType, var_no: int, value: Any, persistent: bool = False) -> bool:
        """Write a single variable (LOADV). persistent=True: call inside existing session."""
        data = self._var_payload(var_type, var_no, value)
        raw = self._exec(persistent, "LOADV", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def write_variable_block(self, var_tuples: list[tuple]) -> bool:
        """
        Write N variables in one TCP session (one send_start, N LOADV commands).
        var_tuples: [(VarType, var_no, value), ...]
        Call inside a with-block (socket must already be open).
        """
        self.send_start(keep_alive=len(var_tuples))
        for var_type, var_no, value in var_tuples:
            data = self._var_payload(var_type, var_no, value)
            raw = self.send_command("LOADV", size=len(data), data=data)
            if self._clean(raw) != "0000":
                return False
        return True

    def read_variable_block(self, var_tuples: list[tuple]) -> dict:
        """
        Read N variables in one TCP session.
        var_tuples: [(VarType, var_no), ...]
        Call inside a with-block (socket must already be open).
        Returns: {(VarType, var_no): value}
        """
        results: dict = {}
        self.send_start(keep_alive=len(var_tuples))
        for var_type, var_no in var_tuples:
            data = self._var_payload(var_type, var_no)
            raw = self.send_command("SAVEV", size=len(data), data=data)
            results[(var_type, var_no)] = self._parse_var(var_type, self._clean(raw))
        return results

    # ── IO ────────────────────────────────────────────────────────────────────

    def io_read(self, start_no: int, num_points: int, persistent: bool = False) -> list[int]:
        """
        Read IO bits (IOREAD). Returns one integer per 8-bit group.

        start_no:   Yaskawa robot IO address (e.g. 10010 for IN#1, 50010 for OUT#1).
        num_points: Number of bits to read; must be a multiple of 8.
        """
        if num_points % 8:
            raise ValueError("num_points must be a multiple of 8")
        data = f"{start_no},{num_points}\r"
        raw = self._exec(persistent, "IOREAD", size=len(data), data=data, parse=False)
        return [int(v) for v in self._clean(raw).split(",") if v]

    def io_write(self, start_no: int, values: list[int], persistent: bool = False) -> bool:
        """
        Write IO bits (IOWRITE). values: one byte value per 8 contacts.
        NX100: only network inputs #22010–#23287 can be written.
        """
        num_points = len(values) * 8
        data = f"{start_no},{num_points}," + ",".join(str(int(v)) for v in values) + "\r"
        raw = self._exec(persistent, "IOWRITE", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def read_io_bit(self, address: int, persistent: bool = False) -> bool:
        """
        Read a single IO bit by Yaskawa robot address (e.g. 50023, 80023).
        Uses IOREAD on the 8-bit block containing the address.
        Yaskawa address format: last digit = bit index (0-7), rest = byte base.
        """
        bit = address % 10
        base = address - bit
        values = self.io_read(base, 8, persistent=persistent)
        return bool((values[0] >> bit) & 1)

    # ── Status ────────────────────────────────────────────────────────────────

    def read_rstats(self, decode: bool = True, persistent: bool = False) -> dict[str, bool] | list[int]:
        """
        Read robot status (RSTATS).

        decode=True  → dict with named booleans (recommended)
        decode=False → raw [data1, data2] integers

        data1 bits  D0=step  D1=1cycle  D2=auto  D3=running  D4=speed_limit
                    D5=teach  D6=play  D7=cmd_remote
        data2 bits  D1=hold_pendant  D2=hold_external  D3=hold_cmd
                    D4=alarm  D5=error  D6=servo_on
        """
        raw = self._exec(persistent, "RSTATS")
        if not decode:
            return raw  # type: ignore[return-value]
        if isinstance(raw, str):
            raw = [int(p) for p in self._clean(raw).split(",") if p]
        return self._decode_rstats(raw)  # type: ignore[arg-type]

    @staticmethod
    def _decode_rstats(data: list[int]) -> dict[str, bool]:
        d1, d2 = data[0], data[1]
        return {
            "step":          bool(d1 & (1 << 0)),
            "one_cycle":     bool(d1 & (1 << 1)),
            "auto":          bool(d1 & (1 << 2)),
            "running":       bool(d1 & (1 << 3)),
            "speed_limit":   bool(d1 & (1 << 4)),
            "teach":         bool(d1 & (1 << 5)),
            "play":          bool(d1 & (1 << 6)),
            "cmd_remote":    bool(d1 & (1 << 7)),
            "hold_pendant":  bool(d2 & (1 << 1)),
            "hold_external": bool(d2 & (1 << 2)),
            "hold_cmd":      bool(d2 & (1 << 3)),
            "alarm":         bool(d2 & (1 << 4)),
            "error":         bool(d2 & (1 << 5)),
            "servo_on":      bool(d2 & (1 << 6)),
        }

    def read_alarm(self, persistent: bool = False) -> list[int]:
        """
        Read error/alarm codes (RALARM).
        Returns [error_code, error_data, alarm1_code, alarm1_data, ...alarm4_code, alarm4_data].
        """
        raw = self._exec(persistent, "RALARM")
        return raw if isinstance(raw, list) else []  # type: ignore[return-value]

    def read_position_joint(self, persistent: bool = False) -> list[int]:
        """Read current joint position in pulses (RPOSJ). Returns [S, L, U, R, B, T, ...]."""
        raw = self._exec(persistent, "RPOSJ")
        return raw if isinstance(raw, list) else []  # type: ignore[return-value]

    def read_position_cartesian(
        self,
        coord: int = 0,
        external_axis: bool = False,
        persistent: bool = False,
    ) -> dict:
        """
        Read current Cartesian position (RPOSC).
        coord: 0=base, 1=robot, 2-25=user coordinate 1-24.
        Returns: {X, Y, Z, Rx, Ry, Rz, Type, ToolNo}
        """
        data = f"{coord},{1 if external_axis else 0}\r"
        raw = self._exec(persistent, "RPOSC", size=len(data), data=data)
        return raw if isinstance(raw, dict) else {}

    def read_job_seq(self, persistent: bool = False) -> dict:
        """Read current job name, line No., step No. (RJSEQ). Returns {JobName, Line, Step}."""
        raw = self._exec(persistent, "RJSEQ")
        return raw if isinstance(raw, dict) else {}

    def list_jobs(self, parent_job: str = "*", persistent: bool = False) -> list[str]:
        """
        List registered job names (RJDIR).
        parent_job='*' → all jobs; pass a job name to list its child jobs.
        """
        data = f"{parent_job}\r"
        raw = self._exec(persistent, "RJDIR", size=len(data), data=data, parse=False)
        return [n for n in self._clean(raw).split(",") if n]

    # ── Control commands ──────────────────────────────────────────────────────

    def servo_on(self, on: bool = True, persistent: bool = False) -> bool:
        """Set servo power (SVON). on=True → ON, on=False → OFF."""
        data = f"{1 if on else 0}\r"
        raw = self._exec(persistent, "SVON", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def hold(self, on: bool = True, persistent: bool = False) -> bool:
        """Set hold (HOLD). on=True → hold on, on=False → release hold."""
        data = f"{1 if on else 0}\r"
        raw = self._exec(persistent, "HOLD", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def reset(self, persistent: bool = False) -> bool:
        """Reset alarm (RESET). Transmission alarms can only be reset from the programming pendant."""
        raw = self._exec(persistent, "RESET", parse=False)
        return self._clean(raw).startswith("0000")

    def cancel(self, persistent: bool = False) -> bool:
        """Cancel error (CANCEL)."""
        raw = self._exec(persistent, "CANCEL", parse=False)
        return self._clean(raw).startswith("0000")

    def set_mode(self, mode: int, persistent: bool = False) -> bool:
        """
        Set operation mode (MODE). Requires external mode switch enabled on controller.
        mode: 1=Teach, 2=Play
        """
        data = f"{mode}\r"
        raw = self._exec(persistent, "MODE", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def set_cycle(self, cycle: int, persistent: bool = False) -> bool:
        """
        Set cycle mode (CYCLE).
        cycle: 1=Step, 2=1cycle, 3=Auto
        """
        data = f"{cycle}\r"
        raw = self._exec(persistent, "CYCLE", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def hlock(self, on: bool = True, persistent: bool = False) -> bool:
        """
        Set programming-pendant interlock (HLOCK).
        on=True: locks out pendant and I/O except emergency stop.
        """
        data = f"{1 if on else 0}\r"
        raw = self._exec(persistent, "HLOCK", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def start_job(self, jobname: str | None = None, persistent: bool = False) -> bool:
        """
        Start a job (START). Servo must be ON before calling with a job name.
        jobname=None → start from current program pointer.
        jobname='FOO' → start FOO from the beginning (no .JBI extension).
        """
        data = f"{jobname}\r" if jobname else "\r"
        raw = self._exec(persistent, "START", size=len(data), data=data, parse=False)
        return self._clean(raw).startswith("0000")

    def set_master_job(self, jobname: str, persistent: bool = False) -> bool:
        """
        Set master job permanently in the controller (SETMJ).
        Also sets it as the current execution job.
        Requires Remote/Play mode; error 2100 otherwise.
        """
        data = f"{jobname}\r"
        raw = self._exec(persistent, "SETMJ", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def get_master_job(self, persistent: bool = False) -> str | None:
        """Read the currently set master job name (GETMJ). Returns None on error."""
        raw = self._exec(persistent, "GETMJ", parse=False)
        result = self._clean(raw)
        return result if result else None

    def set_job_seq(self, jobname: str, line: int, persistent: bool = False) -> bool:
        """Set job name and execution line number (JSEQ)."""
        data = f"{jobname},{line}\r"
        raw = self._exec(persistent, "JSEQ", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def display_message(self, message: str, persistent: bool = False) -> bool:
        """
        Display a message on the programming-pendant remote display (MDSP).
        Maximum 30 bytes; truncated silently if longer.
        """
        data = f"{message[:30]}\r"
        raw = self._exec(persistent, "MDSP", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def set_control_group(self, robot_bits: int, station_bits: int, persistent: bool = False) -> bool:
        """
        Change the target control group (CGROUP).
        robot_bits: bitmask of robots (D0=R1, D1=R2, D2=R3, D3=R4).
        station_bits: bitmask of stations (D0=S1 … D11=S12).
        """
        data = f"{robot_bits},{station_bits}\r"
        raw = self._exec(persistent, "CGROUP", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    def delete_job(self, jobname: str = "*", persistent: bool = False) -> bool:
        """
        Delete a job (DELETE). jobname='*' deletes ALL registered jobs.
        """
        data = f"{jobname}\r"
        raw = self._exec(persistent, "DELETE", size=len(data), data=data, parse=False)
        return self._clean(raw) == "0000"

    # ── IO address helpers ────────────────────────────────────────────────────

    @staticmethod
    def io_addr_to_byte_bit(rob_addr: int) -> tuple[int, int]:
        """
        Split a Yaskawa robot IO address into (byte_base, bit_index).
        Each Yaskawa IO byte spans 10 address units; the last digit is the bit.

        Examples:
          50053 → (50050, 3)   bit 3 of byte at 50050
          10015 → (10010, 5)   bit 5 of byte at 10010
          50020 → (50020, 0)   bit 0 of byte at 50020
        """
        bit = rob_addr % 10
        return rob_addr - bit, bit


# ── Backwards-compatible alias ────────────────────────────────────────────────

EthernetServerClient = YaskawaClient
