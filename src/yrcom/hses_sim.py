"""
HSES simulator — replays a recorded robot session (tx/rx wire pairs) as a
UDP server so the **real** HsesClient can work against it offline.

The server indexes the recorded request→response byte pairs. It ignores the
request ID (header byte 11), which increments per call, and patches it back
into the response — so the response matches any request regardless of the
order in which the client asks. The exact wire bytes of the real controller
are sent back, so the real parser in HsesClient runs unchanged.

Usage:

    # as a context manager (thread, ephemeral or fixed port)
    from yrcom import HsesReplayServer, HsesClient
    with HsesReplayServer.from_capture("captures/latest.json") as srv:
        c = HsesClient("127.0.0.1", srv.port, 2.0); c.open()
        print(c.read_status())        # real recorded response, offline

    # or standalone on the default port 10040:
    #   python -m yrcom.hses_sim captures/latest.json --port 10040

Limits: this is a replay, not a state machine. A request not present in the
capture is answered with HSES status 0x01 ("not executable") — the same reply
the real controller gives for unknown/invalid reads. Writes change nothing; a
subsequent read still returns the recorded value.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
from pathlib import Path
from typing import Optional

_HDR_SIZE = 0x0020
_REQ_ID_OFF = 11        # header byte that increments per request
_MAGIC = b"YERC"


def _req_key(pkt: bytes) -> bytes:
    """Canonical key of a request: request ID masked out."""
    b = bytearray(pkt)
    if len(b) > _REQ_ID_OFF:
        b[_REQ_ID_OFF] = 0
    return bytes(b)


def _error_response(req: bytes, status: int = 0x01) -> bytes:
    """Build a valid HSES error response (status != 0) to a request.

    Mirrors the header, sets the ACK flag, status and empty data length —
    exactly the shape the real controller sends for a non-executable read.
    """
    resp = bytearray(_HDR_SIZE)
    if len(req) >= _HDR_SIZE:
        resp[:] = req[:_HDR_SIZE]
    else:
        resp[0:4] = _MAGIC
        struct.pack_into("<H", resp, 4, _HDR_SIZE)
    struct.pack_into("<H", resp, 6, 0)   # data size = 0
    resp[10] = 0x01                        # ACK = answer
    resp[26] = status                      # status byte
    resp[27] = 0
    resp[28] = 0                           # added-status size
    struct.pack_into("<H", resp, 30, 0)
    return bytes(resp)


class HsesReplayServer:
    """UDP server that replays recorded HSES responses."""

    def __init__(self, table: dict[bytes, bytes], host: str = "127.0.0.1",
                 port: int = 0) -> None:
        self._table = table
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self.host, self.port = self._sock.getsockname()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.hits = 0
        self.misses = 0

    # ── construction from a capture file ──────────────────────────────────────
    @classmethod
    def from_capture(cls, path: str | Path, host: str = "127.0.0.1",
                     port: int = 0) -> "HsesReplayServer":
        data = json.loads(Path(path).read_text())
        table: dict[bytes, bytes] = {}
        hses = data.get("hses", {})
        for op in hses.get("ops", []):
            for ex in op.get("wire", []):
                tx, rx = ex.get("tx"), ex.get("rx")
                if not tx or not rx:
                    continue
                table[_req_key(bytes.fromhex(tx))] = bytes.fromhex(rx)
        if not table:
            raise ValueError(f"no HSES wire data in {path}")
        return cls(table, host, port)

    # ── server loop ───────────────────────────────────────────────────────────
    def _serve(self) -> None:
        self._sock.settimeout(0.25)
        while self._running:
            try:
                req, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            resp = self._table.get(_req_key(req))
            if resp is None:
                self.misses += 1
                out = _error_response(req)
            else:
                self.hits += 1
                out = bytearray(resp)
                # patch the caller's request ID into the response
                if len(req) > _REQ_ID_OFF and len(out) > _REQ_ID_OFF:
                    out[_REQ_ID_OFF] = req[_REQ_ID_OFF]
                out = bytes(out)
            try:
                self._sock.sendto(out, addr)
            except OSError:
                break

    def start(self) -> "HsesReplayServer":
        if self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self) -> "HsesReplayServer":
        return self.start()

    def __exit__(self, *_exc) -> None:
        self.stop()


def _main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="HSES replay server from a capture file")
    ap.add_argument("capture", nargs="?", default="captures/latest.json")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=10040)
    args = ap.parse_args()

    srv = HsesReplayServer.from_capture(args.capture, args.host, args.port).start()
    print(f"HSES simulator listening on {srv.host}:{srv.port} "
          f"({len(srv._table)} responses from {args.capture})")
    print("Stop with Ctrl-C")
    try:
        while True:
            srv._thread.join(1.0)
    except KeyboardInterrupt:
        print(f"\nhits={srv.hits} misses={srv.misses}")
        srv.stop()


if __name__ == "__main__":
    _main()
