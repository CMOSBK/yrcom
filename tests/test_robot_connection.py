"""
Unit tests for the RobotConnection facade (yrcom/robot_connection.py).

No real robot and no sockets: the two underlying transport clients (HsesClient,
YaskawaClient) are replaced with in-memory fakes. These tests cover the facade
logic — transport selection, lazy client creation, status routing, dual-probe
detection and validation — not the wire protocols (those are tested elsewhere).
"""
from __future__ import annotations

import pytest

import yrcom.robot_connection as rc
from yrcom.robot_connection import (
    RobotConnection,
    TRANSPORT_ETHERNET,
    TRANSPORT_HSES,
)


# ── Fakes ─────────────────────────────────────────────────────────────────────


def make_fake_hses(ok=True):
    """Fake HsesClient. `ok=False` makes read_status raise (unreachable)."""

    class FakeHses:
        created = []

        def __init__(self, host, port=10040, timeout=2.0, bind_host=None, model="yrc1000"):
            self.host = host
            self.port = port
            self.opened = False
            self.closed = False
            FakeHses.created.append(self)

        def open(self):
            self.opened = True

        def close(self):
            self.closed = True

        def read_status(self):
            if not ok:
                raise ConnectionError("hses unreachable")
            return {"servo_on": True, "running": False, "_via": "hses"}

        def detect_model(self):
            return "yrc1000"

    return FakeHses


def make_fake_yaskawa(ok=True):
    """Fake YaskawaClient (TCP Ethernet Server). `ok=False` makes reads raise."""

    class FakeYaskawa:
        created = []

        def __init__(self, host, port=80, timeout=5.0):
            self.host = host
            self.port = port
            self.connected = False
            self.closed = False
            self.keep_alive = None
            FakeYaskawa.created.append(self)

        def connect(self):
            if not ok:
                raise ConnectionError("tcp unreachable")
            self.connected = True

        def send_start(self, keep_alive=None):
            self.keep_alive = keep_alive
            return "OK"

        def close(self):
            self.closed = True

        def read_rstats(self, decode=True, persistent=False):
            if not ok:
                raise ConnectionError("tcp unreachable")
            return {"servo_on": False, "running": True, "_via": "ethernet"}

    return FakeYaskawa


@pytest.fixture
def patch_transports(monkeypatch):
    """Install both fakes (both reachable) and hand them back for assertions."""
    hses = make_fake_hses(ok=True)
    tcp = make_fake_yaskawa(ok=True)
    monkeypatch.setattr(rc, "HsesClient", hses)
    monkeypatch.setattr(rc, "YaskawaClient", tcp)
    return hses, tcp


# ── Construction / configuration ──────────────────────────────────────────────


def test_from_settings_defaults_to_hses():
    conn = RobotConnection.from_settings({"robot_ip": "10.0.0.5"})
    assert conn.transport == TRANSPORT_HSES
    assert conn.ip == "10.0.0.5"
    assert conn.tcp_port == 80
    assert conn.hses_port == 10040


def test_from_settings_reads_transport_and_ports():
    conn = RobotConnection.from_settings(
        {"robot_ip": "10.0.0.5", "robot_port": 8080, "hses_port": 10041, "transport": "ethernet"}
    )
    assert conn.transport == TRANSPORT_ETHERNET
    assert conn.tcp_port == 8080
    assert conn.hses_port == 10041


def test_invalid_transport_rejected():
    with pytest.raises(ValueError):
        RobotConnection("1.2.3.4", transport="carrier-pigeon")


# ── connect() opens the chosen transport ──────────────────────────────────────


def test_connect_hses_opens_hses_client(patch_transports):
    hses, tcp = patch_transports
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_HSES)
    status = conn.connect()

    assert status["_via"] == "hses"
    assert conn.connected is True
    assert len(hses.created) == 1 and hses.created[0].opened is True
    assert tcp.created == []  # TCP path untouched


def test_connect_ethernet_opens_tcp_session(patch_transports):
    hses, tcp = patch_transports
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_ETHERNET)
    status = conn.connect()

    assert status["_via"] == "ethernet"
    assert conn.connected is True
    # PersistentTcpClient established a keep-alive session
    assert len(tcp.created) == 1
    assert tcp.created[0].connected is True
    assert tcp.created[0].keep_alive == -1
    assert hses.created == []  # HSES path untouched


# ── status() routes over the active transport ─────────────────────────────────


def test_status_uses_active_transport(patch_transports):
    hses_conn = RobotConnection("1.2.3.4", transport=TRANSPORT_HSES)
    assert hses_conn.status()["_via"] == "hses"

    eth_conn = RobotConnection("1.2.3.4", transport=TRANSPORT_ETHERNET)
    assert eth_conn.status()["_via"] == "ethernet"


# ── both clients stay reachable regardless of active transport ────────────────


def test_hses_available_in_ethernet_mode(patch_transports):
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_ETHERNET)
    # The non-active transport is still usable on demand (lazy).
    assert conn.hses.read_status()["_via"] == "hses"


def test_tcp_available_in_hses_mode(patch_transports):
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_HSES)
    assert conn.tcp.read_rstats()["_via"] == "ethernet"


# ── detect() probes both transports independently ─────────────────────────────


def test_detect_both_reachable(monkeypatch):
    monkeypatch.setattr(rc, "HsesClient", make_fake_hses(ok=True))
    monkeypatch.setattr(rc, "YaskawaClient", make_fake_yaskawa(ok=True))
    conn = RobotConnection("1.2.3.4")
    assert conn.detect() == {"hses": True, "ethernet": True}


def test_detect_only_hses(monkeypatch):
    monkeypatch.setattr(rc, "HsesClient", make_fake_hses(ok=True))
    monkeypatch.setattr(rc, "YaskawaClient", make_fake_yaskawa(ok=False))
    conn = RobotConnection("1.2.3.4")
    assert conn.detect() == {"hses": True, "ethernet": False}


def test_detect_only_ethernet(monkeypatch):
    monkeypatch.setattr(rc, "HsesClient", make_fake_hses(ok=False))
    monkeypatch.setattr(rc, "YaskawaClient", make_fake_yaskawa(ok=True))
    conn = RobotConnection("1.2.3.4")
    assert conn.detect() == {"hses": False, "ethernet": True}


def test_detect_neither(monkeypatch):
    monkeypatch.setattr(rc, "HsesClient", make_fake_hses(ok=False))
    monkeypatch.setattr(rc, "YaskawaClient", make_fake_yaskawa(ok=False))
    conn = RobotConnection("1.2.3.4")
    assert conn.detect() == {"hses": False, "ethernet": False}


# ── connect() failure surfaces and leaves us disconnected ─────────────────────


def test_connect_hses_failure_propagates(monkeypatch):
    hses = make_fake_hses(ok=False)
    monkeypatch.setattr(rc, "HsesClient", hses)
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_HSES)
    with pytest.raises(ConnectionError):
        conn.connect()
    assert conn.connected is False
    assert hses.created[0].closed is True  # opened client was cleaned up


# ── disconnect() tears down what was opened ───────────────────────────────────


def test_disconnect_closes_clients(patch_transports):
    hses, tcp = patch_transports
    conn = RobotConnection("1.2.3.4", transport=TRANSPORT_HSES)
    conn.connect()
    _ = conn.tcp.read_rstats()  # also spin up the TCP side
    conn.disconnect()

    assert conn.connected is False
    assert hses.created[0].closed is True
    assert tcp.created[0].closed is True
