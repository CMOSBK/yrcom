"""Additional Yaskawa SDK tests — robot control commands and IO via mock socket."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from yrcom.yaskawa_client import VarType, YaskawaClient


def _sock(*responses: bytes) -> MagicMock:
    s = MagicMock()
    s.recv.side_effect = list(responses)
    return s


def _patch(mock_sock: MagicMock):
    return patch("yrcom.yaskawa_client.socket.create_connection", return_value=mock_sock)


# ── IO read / write ───────────────────────────────────────────────────────────

def test_io_read_returns_bytes():
    sock = _sock(
        b"OK: NX\r\n",
        b"OK: IOREAD\r\n",
        b"170,85\r",          # 0b10101010, 0b01010101
    )
    with _patch(sock):
        client = YaskawaClient("x")
        result = client.io_read(50050, 16)
    assert result == [170, 85]


def test_io_read_num_points_must_be_multiple_of_8():
    client = YaskawaClient("x")
    with pytest.raises(ValueError):
        client.io_read(50050, 7)


def test_io_write_success():
    sock = _sock(b"OK: NX\r\n", b"OK: IOWRITE\r\n", b"0000\r")
    with _patch(sock):
        ok = YaskawaClient("x").io_write(50050, [255])
    assert ok is True


def test_io_write_failure():
    sock = _sock(b"OK: NX\r\n", b"OK: IOWRITE\r\n", b"1011\r")
    with _patch(sock):
        ok = YaskawaClient("x").io_write(50050, [255])
    assert ok is False


def test_read_io_bit_true():
    # address 50053 → base=50050, bit=3; byte value 0b00001000 = 8 → bit3 = True
    sock = _sock(b"OK: NX\r\n", b"OK: IOREAD\r\n", b"8\r")
    with _patch(sock):
        result = YaskawaClient("x").read_io_bit(50053)
    assert result is True


def test_read_io_bit_false():
    sock = _sock(b"OK: NX\r\n", b"OK: IOREAD\r\n", b"0\r")
    with _patch(sock):
        result = YaskawaClient("x").read_io_bit(50053)
    assert result is False


# ── Control commands ──────────────────────────────────────────────────────────

def test_servo_on():
    sock = _sock(b"OK: NX\r\n", b"OK: SVON\r\n", b"0000\r")
    with _patch(sock):
        assert YaskawaClient("x").servo_on(True) is True


def test_servo_off():
    sock = _sock(b"OK: NX\r\n", b"OK: SVON\r\n", b"0000\r")
    with _patch(sock):
        assert YaskawaClient("x").servo_on(False) is True


def test_hold_on():
    sock = _sock(b"OK: NX\r\n", b"OK: HOLD\r\n", b"0000\r")
    with _patch(sock):
        assert YaskawaClient("x").hold(True) is True


def test_reset_alarm():
    sock = _sock(b"OK: NX\r\n", b"OK: RESET\r\n", b"0000\r")
    with _patch(sock):
        assert YaskawaClient("x").reset() is True


def test_cancel_error():
    sock = _sock(b"OK: NX\r\n", b"OK: CANCEL\r\n", b"0000\r")
    with _patch(sock):
        assert YaskawaClient("x").cancel() is True


def test_start_job_with_name():
    sock = _sock(b"OK: NX\r\n", b"OK: START\r\n", b"0000\r")
    with _patch(sock):
        ok = YaskawaClient("x").start_job("MASTERJOB")
    assert ok is True
    # Verify job name was sent in the payload
    calls = sock.sendall.call_args_list
    sent = b"".join(c.args[0] for c in calls)
    assert b"MASTERJOB" in sent


def test_start_job_error_2070():
    sock = _sock(b"OK: NX\r\n", b"OK: START\r\n", b"2070\r")
    with _patch(sock):
        ok = YaskawaClient("x").start_job("JOB")
    assert ok is False


def test_start_job_without_name():
    sock = _sock(b"OK: NX\r\n", b"OK: START\r\n", b"0000\r")
    with _patch(sock):
        ok = YaskawaClient("x").start_job()
    assert ok is True


def test_set_master_job():
    sock = _sock(b"OK: NX\r\n", b"OK: SETMJ\r\n", b"0000\r")
    with _patch(sock):
        ok = YaskawaClient("x").set_master_job("MASTERJOB")
    assert ok is True


def test_get_master_job():
    sock = _sock(b"OK: NX\r\n", b"OK: GETMJ\r\n", b"MASTERJOB\r")
    with _patch(sock):
        name = YaskawaClient("x").get_master_job()
    assert name == "MASTERJOB"


def test_get_master_job_empty():
    sock = _sock(b"OK: NX\r\n", b"OK: GETMJ\r\n", b"\r")
    with _patch(sock):
        name = YaskawaClient("x").get_master_job()
    assert name is None


# ── Read variable ─────────────────────────────────────────────────────────────

def test_read_variable_byte():
    sock = _sock(b"OK: NX\r\n", b"OK: SAVEV\r\n", b"42\r")
    with _patch(sock):
        val = YaskawaClient("x").read_variable(VarType.BYTE, 10)
    assert val == 42


def test_read_variable_block():
    sock = _sock(
        b"OK: NX\r\n",
        b"OK: SAVEV\r\n", b"10\r",
        b"OK: SAVEV\r\n", b"20\r",
    )
    with _patch(sock):
        with YaskawaClient("x") as client:
            result = client.read_variable_block([
                (VarType.BYTE, 1),
                (VarType.BYTE, 2),
            ])
    assert result[(VarType.BYTE, 1)] == 10
    assert result[(VarType.BYTE, 2)] == 20


# ── Persistent session (send_transaction) ────────────────────────────────────

def test_send_transaction_on_open_session():
    """send_transaction must NOT call send_start — only send_command."""
    sock = _sock(
        b"OK: NX\r\n",       # for explicit send_start
        b"OK: RSTATS\r\n",   # send_command header
        b"8,64\r",           # answer
    )
    with _patch(sock):
        with YaskawaClient("x") as client:
            client.send_start()
            raw = client.send_transaction("RSTATS", parse=False)
    assert "8,64" in raw


# ── Additional status commands ────────────────────────────────────────────────

def test_read_alarm():
    sock = _sock(b"OK: NX\r\n", b"OK: RALARM\r\n", b"4100,0,0,0,0,0,0,0,0,0\r")
    with _patch(sock):
        alarms = YaskawaClient("x").read_alarm()
    assert alarms[0] == 4100


def test_read_position_joint():
    sock = _sock(b"OK: NX\r\n", b"OK: RPOSJ\r\n", b"0,1000,2000,3000,4000,5000\r")
    with _patch(sock):
        pos = YaskawaClient("x").read_position_joint()
    assert pos == [0, 1000, 2000, 3000, 4000, 5000]


def test_read_job_sequence():
    sock = _sock(b"OK: NX\r\n", b"OK: RJSEQ\r\n", b"MYJOB,5,3\r")
    with _patch(sock):
        seq = YaskawaClient("x").read_job_seq()
    assert seq == {"JobName": "MYJOB", "Line": 5, "Step": 3}


def test_list_jobs():
    sock = _sock(b"OK: NX\r\n", b"OK: RJDIR\r\n", b"JOB1,JOB2,MASTERJOB\r")
    with _patch(sock):
        jobs = YaskawaClient("x").list_jobs()
    assert "MASTERJOB" in jobs
    assert len(jobs) == 3
