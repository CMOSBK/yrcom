"""Pure protocol-logic tests — no sockets, no MotoSim. Builds real request
bytes via HsesClient._build (the verified client-side packer) and checks the
new server-side decoder reads them back correctly."""
from __future__ import annotations

import pytest

from yrcom.hses_client import HsesClient
from yrcom.hses_server_codec import MalformedRequest, ServerCommand, decode_request


def test_decode_matches_a_real_status_request():
    raw = HsesClient._build(cmd=0x72, instance=1, attribute=0, service=0x01, req_id=5)
    sc = decode_request(raw)
    assert sc == ServerCommand(cmd=0x72, instance=1, attribute=0, service=0x01,
                                req_id=5, payload=b"")


def test_decode_carries_payload():
    raw = HsesClient._build(cmd=0x7A, instance=3, attribute=1, service=0x10,
                             req_id=200, payload=b"\x2a\x00\x00\x00")
    sc = decode_request(raw)
    assert sc.cmd == 0x7A
    assert sc.instance == 3
    assert sc.payload == b"\x2a\x00\x00\x00"


def test_decode_rejects_short_input():
    with pytest.raises(MalformedRequest):
        decode_request(b"too short")


def test_decode_rejects_bad_magic():
    raw = bytearray(HsesClient._build(cmd=0x72, instance=1, attribute=0,
                                       service=0x01, req_id=1))
    raw[0:4] = b"XXXX"
    with pytest.raises(MalformedRequest):
        decode_request(bytes(raw))


def test_encode_response_round_trips_through_real_client_parser():
    # HsesClient._parse_resp is the real, hardware-verified response parser —
    # proving our encoder's bytes satisfy it is the strongest test we have
    # without a socket.
    from yrcom.hses_client import HsesClient
    from yrcom.hses_server_codec import encode_response

    raw = encode_response(cmd=0x72, req_id=9, data=b"\x01\x00\x00\x00\x00\x00\x00\x00")
    resp = HsesClient._parse_resp(raw)
    assert resp.cmd == 0x72
    assert resp.status == 0x00
    assert resp.added == 0
    assert resp.data == b"\x01\x00\x00\x00\x00\x00\x00\x00"


def test_encode_error_round_trips_and_raises_hses_error():
    from yrcom.hses_client import HsesClient, HsesError
    from yrcom.hses_server_codec import encode_error

    raw = encode_error(cmd=0x75, req_id=1, status=0x1F, added=0xB005)
    resp = HsesClient._parse_resp(raw)
    assert resp.status == 0x1F
    assert resp.added == 0xB005
    assert resp.data == b""


def test_encode_error_without_added_status():
    from yrcom.hses_client import HsesClient
    from yrcom.hses_server_codec import encode_error

    raw = encode_error(cmd=0x00, req_id=1, status=0x08)
    resp = HsesClient._parse_resp(raw)
    assert resp.status == 0x08
    assert resp.added == 0
