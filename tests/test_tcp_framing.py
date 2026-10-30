"""Ethernet Server replies arrive as a TCP byte stream: a header and its answer may come
in one segment, a line may be split over several, and ``\\r\\n`` may be cut between
``\\r`` and ``\\n``. The client must read lines, not whatever one recv() returns."""
from __future__ import annotations

import socket
import threading
import time

import pytest

from yrcom.robot_connection import PersistentTcpClient
from yrcom.yaskawa_client import YaskawaClient

GREETING = b"OK: NX Information Server(Ver. W4.00)\r\n"


def _serve(chunks_for_request, keep=0.3):
    """One-connection server: answers CONNECT with GREETING, then every request header
    with the chunks `chunks_for_request(n)` returns (n = request number, from 0)."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)

    def run():
        conn, _ = srv.accept()
        f = conn.makefile("rb")
        f.readline()                                  # CONNECT Robot_access ...
        conn.sendall(GREETING)
        n = 0
        while True:
            if not f.readline():                      # HOSTCTRL_REQUEST ...
                break
            for c in chunks_for_request(n):
                conn.sendall(c)
                time.sleep(0.005)                     # separate segments where intended
            n += 1
        time.sleep(keep)
        conn.close()

    threading.Thread(target=run, daemon=True).start()
    return srv, srv.getsockname()[1]


REPLY = b"OK: RSTATS\r\n" + b"194,64\r"


def _bytewise(data: bytes) -> list[bytes]:
    return [bytes([b]) for b in data]


@pytest.mark.parametrize("chunks", [
    [b"OK: RSTATS\r\n", b"194,64\r"],                         # header and answer apart
    [REPLY],                                                  # header and answer in one segment
    _bytewise(REPLY),                                         # one byte per segment
    [b"OK: RSTATS\r", b"\n194,", b"64\r"],                    # \r\n cut between \r and \n
    [b"OK: RSTATS\r\n194,64\r\n"],                            # answer terminated with \r\n
], ids=["apart", "together", "bytewise", "crlf-split", "answer-crlf"])
def test_rstats_survives_any_segmentation(chunks):
    srv, port = _serve(lambda n: chunks)
    try:
        with YaskawaClient("127.0.0.1", port, timeout=2.0) as c:
            c.send_start(keep_alive=-1)
            assert c.read_rstats(decode=False, persistent=True) == [194, 64]
    finally:
        srv.close()


def test_two_replies_in_one_segment_stay_apart():
    # first request: both replies of the session arrive at once; the second request must
    # get its own answer from the buffer, not wait for the network
    both = b"OK: RSTATS\r\n1,2\r" + b"OK: RSTATS\r\n3,4\r"
    srv, port = _serve(lambda n: [both] if n == 0 else [])
    try:
        with YaskawaClient("127.0.0.1", port, timeout=2.0) as c:
            c.send_start(keep_alive=-1)
            assert c.read_rstats(decode=False, persistent=True) == [1, 2]
            assert c.read_rstats(decode=False, persistent=True) == [3, 4]
    finally:
        srv.close()


def test_ng_header_returns_without_waiting():
    srv, port = _serve(lambda n: [b"NG: HOSTCTRL_REQUEST\r\n"])
    try:
        with YaskawaClient("127.0.0.1", port, timeout=2.0) as c:
            c.send_start(keep_alive=-1)
            t0 = time.monotonic()
            assert c.send_command("RSTATS").startswith("NG")
            assert time.monotonic() - t0 < 1.0          # no timeout wait for an answer line
    finally:
        srv.close()


def test_persistent_connection_together_and_bytewise():
    for chunks in ([REPLY], _bytewise(REPLY)):
        srv, port = _serve(lambda n, ch=chunks: ch)
        try:
            tcp = PersistentTcpClient("127.0.0.1", port, timeout=2.0)
            assert tcp.read_rstats(decode=False) == [194, 64]
            tcp.close()
        finally:
            srv.close()
