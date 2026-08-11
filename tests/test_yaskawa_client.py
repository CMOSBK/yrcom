"""Unit tests for the standalone Yaskawa SDK (yaskawa_client.py)."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from yrcom.yaskawa_client import (
    DEFAULT_PORT,
    VarType,
    YaskawaClient,
)


# ── VarType enum ──────────────────────────────────────────────────────────────

def test_vartype_values():
    assert VarType.BYTE       == 0
    assert VarType.INTEGER    == 1
    assert VarType.DOUBLE     == 2
    assert VarType.FLOAT      == 3
    assert VarType.ROBOT_POS  == 4
    assert VarType.BASE_POS   == 5
    assert VarType.STATION_POS == 6
    assert VarType.STRING     == 7


# ── Static helpers ────────────────────────────────────────────────────────────

def test_clean():
    assert YaskawaClient._clean("0000\r\n") == "0000"
    assert YaskawaClient._clean("1,64\r")   == "1,64"
    assert YaskawaClient._clean("  ABC  ")  == "ABC"


def test_io_addr_to_byte_bit():
    assert YaskawaClient.io_addr_to_byte_bit(50053) == (50050, 3)
    assert YaskawaClient.io_addr_to_byte_bit(10010) == (10010, 0)
    assert YaskawaClient.io_addr_to_byte_bit(80023) == (80020, 3)
    assert YaskawaClient.io_addr_to_byte_bit(50020) == (50020, 0)


def test_decode_rstats_bits():
    # data1=0b10001000 (bit3=running, bit7=cmd_remote), data2=0b01000000 (bit6=servo_on)
    result = YaskawaClient._decode_rstats([0b10001000, 0b01000000])
    assert result["running"]    is True
    assert result["cmd_remote"] is True
    assert result["servo_on"]   is True
    assert result["step"]       is False
    assert result["alarm"]      is False
    assert result["error"]      is False


def test_decode_rstats_alarm_error():
    # data2 bit4=alarm, bit5=error
    result = YaskawaClient._decode_rstats([0, 0b00110000])
    assert result["alarm"] is True
    assert result["error"] is True
    assert result["servo_on"] is False


@pytest.mark.parametrize("var_type,var_no,value,expected", [
    (VarType.BYTE,    1, 5,     "0,1,5\r"),
    (VarType.INTEGER, 2, 100,   "1,2,100\r"),
    (VarType.DOUBLE,  3, 1000,  "2,3,1000\r"),
    (VarType.FLOAT,   4, 3.14,  "3,4,3.14\r"),
    (VarType.STRING,  1, "ABC", "7,1,ABC\r"),
])
def test_var_payload_write(var_type, var_no, value, expected):
    assert YaskawaClient._var_payload(var_type, var_no, value) == expected


def test_var_payload_read():
    assert YaskawaClient._var_payload(VarType.BYTE, 10)    == "0,10\r"
    assert YaskawaClient._var_payload(VarType.DOUBLE, 5)   == "2,5\r"


def test_var_payload_position():
    payload = YaskawaClient._var_payload(VarType.ROBOT_POS, 1, [100, 200, 300, 0, 0, 0])
    assert payload == "4,1,100,200,300,0,0,0\r"


def test_var_payload_position_requires_list():
    with pytest.raises(ValueError):
        YaskawaClient._var_payload(VarType.ROBOT_POS, 1, 42)


@pytest.mark.parametrize("var_type,raw,expected", [
    (VarType.BYTE,    "42",         42),
    (VarType.INTEGER, "1000",       1000),
    (VarType.DOUBLE,  "99",         99),
    (VarType.FLOAT,   "3.14",       3.14),
    (VarType.STRING,  "HELLO",      "HELLO"),
])
def test_parse_var(var_type, raw, expected):
    result = YaskawaClient._parse_var(var_type, raw)
    assert result == expected


def test_parse_var_invalid_int():
    assert YaskawaClient._parse_var(VarType.BYTE, "??") is None


def test_parse_var_position():
    result = YaskawaClient._parse_var(VarType.ROBOT_POS, "100,200,300,0,0,0")
    assert result == [100, 200, 300, 0, 0, 0]


# ── _parse() response parsing ─────────────────────────────────────────────────

def test_parse_rstats():
    result = YaskawaClient(host="x")._parse("RSTATS", "1,64")
    assert result == [1, 64]


def test_parse_rposj():
    result = YaskawaClient(host="x")._parse("RPOSJ", "0,1000,2000,3000,4000,5000")
    assert result == [0, 1000, 2000, 3000, 4000, 5000]


def test_parse_rposc():
    result = YaskawaClient(host="x")._parse("RPOSC", "100.0,200.0,300.0,0.0,0.0,0.0,0,1")
    assert result["X"] == 100.0
    assert result["ToolNo"] == 1


def test_parse_rjseq():
    result = YaskawaClient(host="x")._parse("RJSEQ", "MYJOB,5,3")
    assert result == {"JobName": "MYJOB", "Line": 5, "Step": 3}


# ── Protocol transaction with mocked socket ───────────────────────────────────

def _make_mock_socket(responses: list[bytes]) -> MagicMock:
    sock = MagicMock()
    sock.recv.side_effect = responses
    return sock


def _patch_connect(mock_sock: MagicMock):
    """Context manager that patches socket.create_connection."""
    return patch("yrcom.yaskawa_client.socket.create_connection", return_value=mock_sock)


def test_read_rstats_via_transaction():
    """Full protocol flow: connect → send_start → RSTATS → decode."""
    mock_sock = _make_mock_socket([
        b"OK: NX Information Server\r\n",  # send_start response
        b"OK: RSTATS\r\n",                  # send_command header ack
        b"8,64\r",                          # RSTATS data: running + servo_on
    ])
    with _patch_connect(mock_sock):
        client = YaskawaClient("192.168.1.1", timeout=1)
        status = client.read_rstats()

    assert status["running"]  is True
    assert status["servo_on"] is True
    assert status["alarm"]    is False
    mock_sock.close.assert_called_once()  # auto-close after single transaction


def test_write_variable_success():
    mock_sock = _make_mock_socket([
        b"OK: NX Information Server\r\n",
        b"OK: LOADV\r\n",
        b"0000\r",
    ])
    with _patch_connect(mock_sock):
        client = YaskawaClient("192.168.1.1", timeout=1)
        ok = client.write_variable(VarType.BYTE, 10, 5)

    assert ok is True


def test_write_variable_failure():
    mock_sock = _make_mock_socket([
        b"OK: NX Information Server\r\n",
        b"OK: LOADV\r\n",
        b"2070\r",  # error: servo off
    ])
    with _patch_connect(mock_sock):
        client = YaskawaClient("192.168.1.1", timeout=1)
        ok = client.write_variable(VarType.BYTE, 10, 5)

    assert ok is False


def test_write_variable_block():
    """One send_start, two LOADV commands on a single connection."""
    mock_sock = _make_mock_socket([
        b"OK: NX Information Server\r\n",  # send_start
        b"OK: LOADV\r\n",                   # cmd1 header ack
        b"0000\r",                          # cmd1 answer
        b"OK: LOADV\r\n",                   # cmd2 header ack
        b"0000\r",                          # cmd2 answer
    ])
    with _patch_connect(mock_sock):
        with YaskawaClient("192.168.1.1", timeout=1) as client:
            ok = client.write_variable_block([
                (VarType.BYTE,   10, 5),
                (VarType.DOUBLE, 1,  1000),
            ])

    assert ok is True


def test_context_manager_auto_close():
    mock_sock = _make_mock_socket([b"OK\r\n"])
    with _patch_connect(mock_sock):
        with YaskawaClient("192.168.1.1") as client:
            assert client.sock is not None
        assert client.sock is None


def test_transaction_reuses_open_socket():
    """Inside a with-block, transaction() must NOT close the socket."""
    mock_sock = _make_mock_socket([
        b"OK: NX Information Server\r\n",  # send_start inside with
        b"OK: RSTATS\r\n",
        b"0,0\r",
    ])
    with _patch_connect(mock_sock):
        with YaskawaClient("192.168.1.1") as client:
            client.transaction("RSTATS")
            assert client.sock is not None  # must still be open


def test_default_port():
    c = YaskawaClient("192.168.1.1")
    assert c.port == DEFAULT_PORT  # 80


def test_connection_timeout_propagates():
    with patch("yrcom.yaskawa_client.socket.create_connection", side_effect=TimeoutError):
        client = YaskawaClient("10.0.0.1", timeout=0.01)
        with pytest.raises(TimeoutError):
            client.connect()
