"""HSES simulator: the real HsesClient must get the same values offline from
the replay server that were recorded.

Needs a capture (`captures/latest.json`, recorded against real hardware).
Without a capture the test is skipped — real plant data deliberately stays
out of the repo (.gitignore).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from yrcom.hses_client import HsesClient
from yrcom.hses_sim import HsesReplayServer

CAPTURE = Path(__file__).resolve().parent.parent / "captures" / "latest.json"

pytestmark = pytest.mark.skipif(
    not CAPTURE.exists(),
    reason="keine Aufnahme (captures/latest.json) — robot_capture.py gegen Hardware laufen lassen",
)


@pytest.fixture
def sim():
    with HsesReplayServer.from_capture(CAPTURE) as srv:
        yield srv


def _decoded(op_name: str):
    data = json.loads(CAPTURE.read_text())
    for op in data["hses"]["ops"]:
        if op["op"] == op_name and op.get("ok"):
            return op["decoded"]
    return None


def test_status_matches_capture(sim):
    c = HsesClient("127.0.0.1", sim.port, 2.0)
    c.open()
    try:
        assert c.read_status() == _decoded("read_status")
    finally:
        c.close()
    assert sim.hits == 1 and sim.misses == 0


def test_job_and_sysinfo_match_capture(sim):
    c = HsesClient("127.0.0.1", sim.port, 2.0)
    c.open()
    try:
        assert c.read_job(1) == _decoded("read_job[1]")
        assert c.read_system_info(11) == _decoded("read_system_info[11]")
    finally:
        c.close()


def test_unrecorded_read_raises_like_controller(sim):
    """An unrecorded read must surface as an HSES error, not hang."""
    from yrcom.hses_client import HsesError

    c = HsesClient("127.0.0.1", sim.port, 2.0)
    c.open()
    try:
        with pytest.raises(HsesError):
            c.read_io(99999)
    finally:
        c.close()
    assert sim.misses >= 1
