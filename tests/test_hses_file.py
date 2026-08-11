"""
Characterisation tests for the HSES File Control client (Yaskawa YRC1000,
UDP 10041). Pins down the wire-protocol behaviour — deliberately WITHOUT any
app coupling: only the public HsesFileClient API against a fake UDP socket.

Reply packets are built with the client's own `_build` static (no hard-coded
header format); status/added bytes are patched in afterwards.
"""
from __future__ import annotations

import socket
import struct

import pytest

from yrcom.hses_file import (
    HsesError,
    HsesFileClient,
    _ACK_ANSWER,
    _ACK_REQUEST,
    _HDR_SIZE,
    _LAST_BLOCK,
    _MAX_DATA,
    _SVC_DELETE,
    _SVC_LIST,
    _SVC_LOAD,
    _SVC_SAVE,
)


# ── Test-Doubles ──────────────────────────────────────────────────────────────

class FakeSocket:
    """UDP-Socket-Ersatz: zeichnet gesendete Pakete auf, gibt vorgereihte
    Antworten zurueck. recv() ohne Antwort -> socket.timeout."""

    def __init__(self, replies: list[bytes] | None = None, close_raises: bool = False):
        self.replies = list(replies or [])
        self.sent: list[tuple[bytes, tuple]] = []
        self.closed = False
        self.bound: tuple | None = None
        self._close_raises = close_raises

    def sendto(self, data, addr):
        self.sent.append((bytes(data), addr))
        return len(data)

    def recv(self, bufsize):
        if not self.replies:
            raise socket.timeout("no more queued replies")
        return self.replies.pop(0)

    def settimeout(self, t):
        pass

    def bind(self, addr):
        self.bound = addr

    def close(self):
        self.closed = True
        if self._close_raises:
            raise OSError("already closed")


def _reply(service: int, block_no: int, data: bytes = b"",
           status: int = 0, added: int = 0, added_size: int = 0) -> bytes:
    """Build a reply packet via the client's own _build static, then patch
    status (raw[26]) and added (raw[28]=size, raw[30:32]=value)."""
    pkt = bytearray(HsesFileClient._build(service, 1, block_no, _ACK_ANSWER, data))
    pkt[26] = status
    pkt[28] = added_size
    struct.pack_into("<H", pkt, 30, added)
    return bytes(pkt)


def _client(replies: list[bytes] | None = None) -> tuple[HsesFileClient, FakeSocket]:
    c = HsesFileClient("10.0.0.9")
    sock = FakeSocket(replies)
    c._sock = sock  # open() umgehen — kein echter Socket noetig
    return c, sock


# ── Request-Paket parsen (fuer Assertions) ────────────────────────────────────

def _parse_sent(entry) -> dict:
    pkt = entry[0] if isinstance(entry, tuple) else entry
    return {
        "ack": pkt[10],
        "block_no": struct.unpack_from("<I", pkt, 12)[0],
        "service": pkt[29],
        "payload": pkt[_HDR_SIZE:],
    }


# ── Lifecycle ─────────────────────────────────────────────────────────────────

def test_open_close_context_manager(monkeypatch):
    # open()/close() live in the shared _HsesSocket base (hses_client), so the
    # socket factory is patched there, not on the hses_file module.
    created = FakeSocket()
    monkeypatch.setattr("yrcom.hses_client.socket.socket", lambda *a, **k: created)

    with HsesFileClient("10.0.0.9") as f:
        assert f._sock is created
    assert created.closed is True
    assert f._sock is None


def test_open_binds_local_host_when_given(monkeypatch):
    created = FakeSocket()
    monkeypatch.setattr("yrcom.hses_client.socket.socket", lambda *a, **k: created)

    f = HsesFileClient("10.0.0.9", bind_host="10.0.0.100")
    f.open()
    assert created.bound == ("10.0.0.100", 0)


def test_close_swallows_socket_error():
    c = HsesFileClient("10.0.0.9")
    c._sock = FakeSocket(close_raises=True)
    c.close()  # must not propagate
    assert c._sock is None


def test_open_is_idempotent(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "yrcom.hses_client.socket.socket",
        lambda *a, **k: calls.append(1) or FakeSocket(),
    )
    f = HsesFileClient("10.0.0.9")
    f.open()
    f.open()
    assert len(calls) == 1


# ── _bump_req_id ──────────────────────────────────────────────────────────────

def test_bump_req_id_wraps_at_255():
    c, _ = _client()
    ids = [c._bump_req_id() for _ in range(256)]
    assert ids[0] == 1
    assert ids[254] == 255
    assert ids[255] == 1  # wrap zurueck auf 1, nie 0


# ── list_files (File List 0x32) ───────────────────────────────────────────────

def test_list_files_single_packet():
    data = b"TESTJOB.JBI\r\nFOO.JBI\r\n\r\n"
    c, sock = _client([_reply(_SVC_LIST, 1 | _LAST_BLOCK, data)])

    names = c.list_files("*.JBI")

    assert names == ["TESTJOB.JBI", "FOO.JBI"]  # Leerzeilen fallen raus
    req = _parse_sent(sock.sent[0])
    assert req["service"] == _SVC_LIST
    assert req["ack"] == _ACK_REQUEST
    assert req["payload"] == b"*.JBI"
    # one ACK for the (final) data packet
    ack = _parse_sent(sock.sent[1])
    assert ack["ack"] == _ACK_ANSWER
    assert ack["block_no"] == (1 | _LAST_BLOCK)


def test_list_files_multi_packet_concatenates_and_acks_each():
    c, sock = _client([
        _reply(_SVC_LIST, 1, b"A.JBI\r\n"),                 # nicht-letztes Paket
        _reply(_SVC_LIST, 2 | _LAST_BLOCK, b"B.JBI\r\n"),   # letztes Paket
    ])

    names = c.list_files("*.JBI")

    assert names == ["A.JBI", "B.JBI"]
    # 1 Request + 2 ACKs (je Datenpaket einer, Blocknummer gespiegelt)
    acks = [_parse_sent(s) for s in sock.sent[1:]]
    assert [a["block_no"] for a in acks] == [1, 2 | _LAST_BLOCK]
    assert all(a["ack"] == _ACK_ANSWER for a in acks)


# ── read_file (File Saving 0x16) ──────────────────────────────────────────────

def test_read_file_returns_concatenated_bytes():
    c, sock = _client([
        _reply(_SVC_SAVE, 1, b"//NOP\r\n"),
        _reply(_SVC_SAVE, 2 | _LAST_BLOCK, b"END\r\n"),
    ])

    assert c.read_file("TESTJOB.JBI") == b"//NOP\r\nEND\r\n"
    req = _parse_sent(sock.sent[0])
    assert req["service"] == _SVC_SAVE
    assert req["payload"] == b"TESTJOB.JBI"


def test_read_file_empty_when_added_status_and_no_data():
    # Status 0, aber added=0xE2A7 (file not found) + leere Daten -> leeres Resultat
    c, sock = _client([
        _reply(_SVC_SAVE, 1 | _LAST_BLOCK, b"", added=0xE2A7, added_size=1),
    ])

    assert c.read_file("MISSING.JBI") == b""
    # no ACK sent — only the request
    assert len(sock.sent) == 1


# ── write_file (File Loading 0x15) ────────────────────────────────────────────

def test_write_file_chunks_content_and_flags_last_packet():
    content = b"X" * (_MAX_DATA + 10)  # zwei Datenpakete
    # Robot bestaetigt: Request-ACK + je Datenpaket ein ACK
    c, sock = _client([
        _reply(_SVC_LOAD, 0),
        _reply(_SVC_LOAD, 1),
        _reply(_SVC_LOAD, 2 | _LAST_BLOCK),
    ])

    c.write_file("NEW.JBI", content)

    req = _parse_sent(sock.sent[0])
    assert req["service"] == _SVC_LOAD
    assert req["ack"] == _ACK_REQUEST
    assert req["payload"] == b"NEW.JBI"
    # zwei Datenpakete, korrekt gechunkt, letztes mit LAST_BLOCK-Flag
    d1, d2 = _parse_sent(sock.sent[1]), _parse_sent(sock.sent[2])
    assert d1["block_no"] == 1
    assert d1["payload"] == b"X" * _MAX_DATA
    assert d2["block_no"] == (2 | _LAST_BLOCK)
    assert d2["payload"] == b"X" * 10


def test_write_file_empty_content_sends_one_last_packet():
    c, sock = _client([_reply(_SVC_LOAD, 0), _reply(_SVC_LOAD, 1 | _LAST_BLOCK)])

    c.write_file("EMPTY.JBI", b"")

    # Request + genau ein Datenpaket, sofort als letztes markiert
    assert len(sock.sent) == 2
    d = _parse_sent(sock.sent[1])
    assert d["block_no"] == (1 | _LAST_BLOCK)
    assert d["payload"] == b""


# ── delete_file (File Deleting 0x09) ──────────────────────────────────────────

def test_delete_file_single_request():
    c, sock = _client([_reply(_SVC_DELETE, 0)])

    c.delete_file("OLD.JBI")

    assert len(sock.sent) == 1
    req = _parse_sent(sock.sent[0])
    assert req["service"] == _SVC_DELETE
    assert req["ack"] == _ACK_REQUEST
    assert req["payload"] == b"OLD.JBI"


def test_delete_file_raises_on_error_status():
    c, _ = _client([_reply(_SVC_DELETE, 0, status=0x1F)])

    with pytest.raises(HsesError):
        c.delete_file("LOCKED.JBI")


# ── backup_cmos ───────────────────────────────────────────────────────────────

def test_backup_cmos_reads_cmos_path():
    c, sock = _client([_reply(_SVC_SAVE, 1 | _LAST_BLOCK, b"\x00\x01\x02")])

    assert c.backup_cmos() == b"\x00\x01\x02"
    req = _parse_sent(sock.sent[0])
    assert req["service"] == _SVC_SAVE
    assert req["payload"] == b"/SPDRV/CMOSBK.BIN"


# ── _recv Fehlerbehandlung ────────────────────────────────────────────────────

def test_recv_raises_on_too_short_reply():
    c, _ = _client([b"tooshort"])
    with pytest.raises(IOError):
        c.delete_file("X.JBI")


def test_recv_raises_on_bad_magic():
    bad = bytearray(_reply(_SVC_DELETE, 0))
    bad[0:4] = b"XXXX"  # falsche Magic
    c, _ = _client([bytes(bad)])
    with pytest.raises(IOError):
        c.delete_file("X.JBI")


# ── Guards: Operation ohne open() ─────────────────────────────────────────────

def test_operations_require_open():
    c = HsesFileClient("10.0.0.9")  # _sock bleibt None
    with pytest.raises(RuntimeError):
        c.read_file("X.JBI")
    with pytest.raises(RuntimeError):
        c.write_file("X.JBI", b"data")
    with pytest.raises(RuntimeError):
        c.delete_file("X.JBI")
