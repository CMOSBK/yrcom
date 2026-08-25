"""
Characterisation tests for the HSES client (Yaskawa YRC1000, UDP 10040).

Same strategy as test_hses_file: no app coupling, pure functions tested
directly, all robot I/O through a fake UDP socket with reply packets built
by the client's own `_build` static.
"""
from __future__ import annotations

import socket
import struct

import pytest

from yrcom import hses_client as hc
from yrcom.hses_client import (
    AxisPositionVar,
    CartesianMove,
    HsesClient,
    HsesError,
    PositionVar,
    PulseMove,
    MODEL_DX100,
    MODEL_DX200,
    MODEL_YRC1000,
    _AXIS_NAMES,
    _pad,
    _decode_alarm,
    _decode_alarm_sub,
    _decode_axis_config,
    _decode_axis_int32,
    _decode_job,
    _decode_management_time,
    _decode_position,
    _decode_regen_power,
    _decode_status,
    _decode_system_info,
    _cstr,
)


# ── Test-Doubles ──────────────────────────────────────────────────────────────

class FakeSocket:
    """Fake UDP socket.

    `echo_req_id=True` (default) puts the request ID of the matching request
    into every reply — this is how the real controller behaves, so tests do
    not have to reproduce the counting themselves. Tests that deliberately
    create the offset set `echo_req_id=False`.
    """

    def __init__(self, replies=None, close_raises=False, echo_req_id=True):
        self.replies = list(replies or [])
        self.sent = []
        self.closed = False
        self.bound = None
        self._close_raises = close_raises
        self._echo_req_id = echo_req_id
        self._unanswered = []          # request IDs in send order

    def sendto(self, data, addr):
        self.sent.append((bytes(data), addr))
        self._unanswered.append(data[11])
        return len(data)

    def recv(self, bufsize):
        if not self.replies:
            raise socket.timeout("no more queued replies")
        raw = bytearray(self.replies.pop(0))
        # Leave deliberately mangled replies untouched — they exercise the
        # length/magic handling and have no ID field at all.
        if self._echo_req_id and self._unanswered and len(raw) > 11:
            raw[11] = self._unanswered.pop(0)
        return bytes(raw)

    def settimeout(self, t):
        pass

    def bind(self, addr):
        self.bound = addr

    def close(self):
        self.closed = True
        if self._close_raises:
            raise OSError("already closed")


def _reply(cmd, data=b"", *, req_id=1, status=0, added=0, added_size=0):
    """Reply packet via the client's own _build static; patch status/added afterwards."""
    pkt = bytearray(HsesClient._build(cmd, 0, 0, hc._SVC_GET_ALL, req_id, data))
    pkt[26] = status
    pkt[28] = added_size
    struct.pack_into("<H", pkt, 30, added)
    return bytes(pkt)


def _client(replies=None, *, model=MODEL_YRC1000, echo_req_id=True):
    c = HsesClient("10.0.0.9", model=model)
    c._sock = FakeSocket(replies, echo_req_id=echo_req_id)
    return c, c._sock


def _parse_req(entry):
    pkt = entry[0] if isinstance(entry, tuple) else entry
    return {
        "req_id": pkt[11],
        "cmd": struct.unpack_from("<H", pkt, 24)[0],
        "instance": struct.unpack_from("<H", pkt, 26)[0],
        "attribute": pkt[28],
        "service": pkt[29],
        "payload": pkt[32:],
    }


# ════════════════════════════════════════════════════════════════════════════
# Reine Datenklassen
# ════════════════════════════════════════════════════════════════════════════

def test_position_var_roundtrip():
    p = PositionVar(data_type=17, figure=1, tool_no=2, user_coord_no=3,
                    extended_figure=4, axes=[10, 20, 30, 40, 50, 60, 70, 80])
    raw = p.to_bytes()
    assert len(raw) == 52
    assert PositionVar.from_bytes(raw) == p


def test_position_var_from_bytes_zero_pads_short_data():
    p = PositionVar.from_bytes(b"")
    assert p.data_type == 0
    assert p.axes == [0] * 8


def test_position_var_to_bytes_rejects_over_length_axes():
    with pytest.raises(ValueError):
        PositionVar(data_type=0, axes=[0] * 9).to_bytes()


def test_axis_position_var_roundtrip():
    a = AxisPositionVar(data_type=16, axes=[1, 2, 3, 4, 5, 6, 7, 8])
    raw = a.to_bytes()
    assert len(raw) == 36
    assert AxisPositionVar.from_bytes(raw) == a


def test_axis_position_var_from_bytes_pads_short_data():
    a = AxisPositionVar.from_bytes(b"")
    assert a.data_type == 0 and a.axes == [0] * 8


def test_axis_position_var_rejects_over_length():
    with pytest.raises(ValueError):
        AxisPositionVar(data_type=0, axes=[0] * 9).to_bytes()


def test_pad_pads_and_rejects_over_length():
    assert _pad([1, 2], 4, "x") == [1, 2, 0, 0]
    assert _pad([1, 2, 3], 3, "x") == [1, 2, 3]
    with pytest.raises(ValueError):
        _pad([1, 2, 3, 4], 3, "x")


def test_cartesian_move_to_bytes_layout():
    m = CartesianMove(x=1000, robot_group=2)
    raw = m.to_bytes()
    assert len(raw) == 104
    fields = struct.unpack_from("<26i", raw)
    assert fields[0] == 2       # robot_group
    assert fields[5] == 1000    # x


def test_pulse_move_to_bytes_layout():
    raw = PulseMove(robot_axes=[1, 2, 3, 4], robot_group=1).to_bytes()
    assert len(raw) == 88


# ── HsesError ─────────────────────────────────────────────────────────────────

def test_hses_error_message_and_attrs():
    e = HsesError(0x72, 0x08, 0)
    assert e.cmd == 0x72 and e.status == 0x08 and e.added == 0
    assert "0x0072" in str(e) and "0x08" in str(e)


def test_hses_error_includes_added_status():
    e = HsesError(0x8A, 0x1F, 0xB008)
    assert "0xB008" in str(e)


# ════════════════════════════════════════════════════════════════════════════
# Model-Logik
# ════════════════════════════════════════════════════════════════════════════

def test_apply_model_yrc1000_uses_32byte_s():
    c, _ = _client(model=MODEL_YRC1000)
    assert c._s_cmd == hc._CMD_S_VAR
    assert c._s_width == 32


def test_apply_model_dx100_uses_16byte_s():
    c, _ = _client(model=MODEL_DX100)
    assert c._s_cmd == hc._CMD_S16_VAR
    assert c._s_width == 16


def test_invalid_model_raises():
    with pytest.raises(ValueError):
        HsesClient("10.0.0.9", model="quantumbot")


def test_model_setter_reapplies():
    c, _ = _client(model=MODEL_YRC1000)
    c.model = MODEL_DX100
    assert c._s_width == 16


def test_require_yrc1000_guard_blocks_dx200():
    c, _ = _client(model=MODEL_DX200)
    with pytest.raises(RuntimeError):
        c.read_converter_temp(1)


# ════════════════════════════════════════════════════════════════════════════
# Transport-Kern
# ════════════════════════════════════════════════════════════════════════════

def test_build_header_structure():
    pkt = HsesClient._build(0x72, 1, 0, hc._SVC_GET_ALL, 7, b"ab")
    assert pkt[0:4] == b"YERC"
    assert struct.unpack_from("<H", pkt, 4)[0] == 0x20   # header size
    assert struct.unpack_from("<H", pkt, 6)[0] == 2      # payload len
    assert pkt[11] == 7                                  # req id
    assert struct.unpack_from("<H", pkt, 24)[0] == 0x72  # cmd
    assert pkt[29] == hc._SVC_GET_ALL
    assert pkt[32:] == b"ab"


def test_bump_req_id_wraps():
    c, _ = _client()
    ids = [c._bump_req_id() for _ in range(256)]
    assert ids[0] == 1 and ids[254] == 255 and ids[255] == 1


def test_tx_requires_open():
    c = HsesClient("10.0.0.9")
    with pytest.raises(RuntimeError):
        c._tx(hc._CMD_STATUS)


def test_tx_success_returns_rawresp():
    c, sock = _client([_reply(hc._CMD_STATUS, b"payload!!")])
    r = c._tx(hc._CMD_STATUS, instance=1, attribute=0, service=hc._SVC_GET_ALL)
    assert r.data == b"payload!!"
    req = _parse_req(sock.sent[0])
    assert req["cmd"] == hc._CMD_STATUS and req["service"] == hc._SVC_GET_ALL


def test_tx_raises_hses_error_on_nonzero_status():
    c, _ = _client([_reply(hc._CMD_STATUS, b"", status=0x08)])
    with pytest.raises(HsesError):
        c._tx(hc._CMD_STATUS)


def test_tx_raises_ioerror_on_short_reply():
    c, _ = _client([b"short"])
    with pytest.raises(IOError):
        c._tx(hc._CMD_STATUS)


def test_tx_raises_ioerror_on_bad_magic():
    bad = bytearray(_reply(hc._CMD_STATUS, b"data"))
    bad[0:4] = b"XXXX"
    c, _ = _client([bytes(bad)])
    with pytest.raises(IOError):
        c._tx(hc._CMD_STATUS)


def test_tx_reraises_timeout():
    c, _ = _client([])  # leerer Reply-Puffer -> recv wirft socket.timeout
    with pytest.raises(socket.timeout):
        c._tx(hc._CMD_STATUS)


# ── detect_model ──────────────────────────────────────────────────────────────

def test_detect_model_yrc1000_when_probe_accepted():
    c, _ = _client([_reply(hc._CMD_S_VAR, b"")])
    assert c.detect_model() == MODEL_YRC1000
    assert c.model == MODEL_YRC1000


def test_detect_model_dx100_on_undefined_status():
    c, _ = _client([_reply(hc._CMD_S_VAR, b"", status=hc._STATUS_CMD_UNDEFINED)])
    assert c.detect_model() == MODEL_DX100
    assert c._s_width == 16


def test_detect_model_yrc1000_on_other_error():
    c, _ = _client([_reply(hc._CMD_S_VAR, b"", status=0x1F)])
    assert c.detect_model() == MODEL_YRC1000


# ════════════════════════════════════════════════════════════════════════════
# Scalar Variable Reads
# ════════════════════════════════════════════════════════════════════════════

def test_read_register():
    c, sock = _client([_reply(hc._CMD_REGISTER, struct.pack("<H", 1234))])
    assert c.read_register(5) == 1234
    req = _parse_req(sock.sent[0])
    assert req["cmd"] == hc._CMD_REGISTER
    assert req["instance"] == 5
    assert req["service"] == hc._SVC_GET_SINGLE


def test_read_register_short_data_returns_zero():
    c, _ = _client([_reply(hc._CMD_REGISTER, b"")])
    assert c.read_register(5) == 0


def test_read_b_var():
    c, _ = _client([_reply(hc._CMD_B_VAR, bytes([42]))])
    assert c.read_b_var(1) == 42


def test_read_i_var_signed():
    c, _ = _client([_reply(hc._CMD_I_VAR, struct.pack("<i", -5))])
    assert c.read_i_var(1) == -5


def test_read_d_var_signed():
    c, _ = _client([_reply(hc._CMD_D_VAR, struct.pack("<i", -100000))])
    assert c.read_d_var(1) == -100000


def test_read_r_var_float():
    c, _ = _client([_reply(hc._CMD_R_VAR, struct.pack("<f", 3.5))])
    assert c.read_r_var(1) == 3.5


def test_read_s_var_strips_nul():
    c, _ = _client([_reply(hc._CMD_S_VAR, b"HELLO" + b"\x00" * 27)])
    assert c.read_s_var(1) == "HELLO"


def test_read_io_returns_first_byte():
    c, sock = _client([_reply(hc._CMD_IO, bytes([1, 0, 0, 0]))])
    assert c.read_io(10010) == 1
    assert _parse_req(sock.sent[0])["instance"] == 1001  # logical // 10


# ── Scalar Variable Writes ────────────────────────────────────────────────────

def test_write_register_payload_and_range():
    c, sock = _client([_reply(hc._CMD_REGISTER)])
    c.write_register(5, 1000)
    req = _parse_req(sock.sent[0])
    assert req["service"] == hc._SVC_SET_SINGLE
    assert struct.unpack_from("<I", req["payload"], 0)[0] == 1000
    with pytest.raises(ValueError):
        c.write_register(5, 70000)


def test_write_b_var_range():
    c, _ = _client([_reply(hc._CMD_B_VAR)])
    with pytest.raises(ValueError):
        c.write_b_var(1, 300)


def test_write_i_var_range():
    c, _ = _client([_reply(hc._CMD_I_VAR)])
    with pytest.raises(ValueError):
        c.write_i_var(1, 40000)


def test_write_d_var_range():
    c, _ = _client([_reply(hc._CMD_D_VAR)])
    with pytest.raises(ValueError):
        c.write_d_var(1, 3_000_000_000)


def test_write_r_var_payload():
    c, sock = _client([_reply(hc._CMD_R_VAR)])
    c.write_r_var(1, 2.5)
    assert struct.unpack_from("<f", _parse_req(sock.sent[0])["payload"], 0)[0] == 2.5


def test_write_s_var_pads_to_width():
    c, sock = _client([_reply(hc._CMD_S_VAR)])
    c.write_s_var(1, "AB")
    payload = _parse_req(sock.sent[0])["payload"]
    assert len(payload) == 32
    assert payload[:2] == b"AB" and payload[2:] == b"\x00" * 30


# ── Position variables (socket round-trip) ────────────────────────────────────

def test_read_position_var_roundtrip():
    p = PositionVar(data_type=17, axes=[1, 2, 3, 4, 5, 6, 7, 8])
    c, sock = _client([_reply(hc._CMD_P_VAR, p.to_bytes())])
    assert c.read_position_var(3) == p
    req = _parse_req(sock.sent[0])
    assert req["cmd"] == hc._CMD_P_VAR and req["service"] == hc._SVC_GET_ALL


def test_write_position_var_payload():
    p = PositionVar(data_type=17, axes=[9, 8, 7, 6, 5, 4, 3, 2])
    c, sock = _client([_reply(hc._CMD_P_VAR)])
    c.write_position_var(3, p)
    assert _parse_req(sock.sent[0])["payload"] == p.to_bytes()


def test_read_and_write_bp_var():
    a = AxisPositionVar(data_type=0, axes=[1, 2, 3, 4, 5, 6, 7, 8])
    c, sock = _client([_reply(hc._CMD_BP_VAR, a.to_bytes()), _reply(hc._CMD_BP_VAR)])
    assert c.read_bp_var(1) == a
    c.write_bp_var(1, a)
    assert _parse_req(sock.sent[1])["payload"] == a.to_bytes()


def test_read_ex_var():
    a = AxisPositionVar(data_type=0, axes=[8, 7, 6, 5, 4, 3, 2, 1])
    c, _ = _client([_reply(hc._CMD_EX_VAR, a.to_bytes())])
    assert c.read_ex_var(1) == a


# ════════════════════════════════════════════════════════════════════════════
# Steuerbefehle
# ════════════════════════════════════════════════════════════════════════════

def test_reset_alarm_and_cancel_error():
    c, sock = _client([_reply(hc._CMD_RESET), _reply(hc._CMD_RESET)])
    c.reset_alarm()
    c.cancel_error()
    r0, r1 = _parse_req(sock.sent[0]), _parse_req(sock.sent[1])
    assert r0["cmd"] == hc._CMD_RESET and r0["instance"] == 1
    assert r1["instance"] == 2


def test_hold_on_off_values():
    c, sock = _client([_reply(hc._CMD_HOLD_SERVO), _reply(hc._CMD_HOLD_SERVO)])
    c.hold(True)
    c.hold(False)
    on = struct.unpack_from("<I", _parse_req(sock.sent[0])["payload"], 0)[0]
    off = struct.unpack_from("<I", _parse_req(sock.sent[1])["payload"], 0)[0]
    assert on == hc._ON and off == hc._OFF
    assert _parse_req(sock.sent[0])["instance"] == hc._HS_HOLD


def test_servo_and_hlock_instances():
    c, sock = _client([_reply(hc._CMD_HOLD_SERVO), _reply(hc._CMD_HOLD_SERVO)])
    c.servo(True)
    c.hlock(True)
    assert _parse_req(sock.sent[0])["instance"] == hc._HS_SERVO
    assert _parse_req(sock.sent[1])["instance"] == hc._HS_HLOCK


def test_set_cycle_valid_and_invalid():
    c, sock = _client([_reply(hc._CMD_CYCLE)])
    c.set_cycle(3)
    assert struct.unpack_from("<I", _parse_req(sock.sent[0])["payload"], 0)[0] == 3
    with pytest.raises(ValueError):
        c.set_cycle(9)


def test_display_message_pads_to_32():
    c, sock = _client([_reply(hc._CMD_MSG)])
    c.display_message("Hallo")
    payload = _parse_req(sock.sent[0])["payload"]
    assert len(payload) == 32 and payload[:5] == b"Hallo"


def test_start_job():
    c, sock = _client([_reply(hc._CMD_JOB_START)])
    c.start_job()
    assert _parse_req(sock.sent[0])["cmd"] == hc._CMD_JOB_START


def test_select_job_instance_and_payload():
    c, sock = _client([_reply(hc._CMD_JOB_SELECT), _reply(hc._CMD_JOB_SELECT)])
    c.select_job("TESTJOB", line=5)
    req = _parse_req(sock.sent[0])
    assert req["instance"] == 1 and req["service"] == hc._SVC_SET_ALL
    assert req["payload"][:7] == b"TESTJOB"
    assert struct.unpack_from("<I", req["payload"], 32)[0] == 5
    c.select_job("J", master_task=3)
    assert _parse_req(sock.sent[1])["instance"] == 13


def test_select_job_validation():
    c, _ = _client([_reply(hc._CMD_JOB_SELECT)])
    with pytest.raises(ValueError):
        c.select_job("J", master_task=99)
    with pytest.raises(ValueError):
        c.select_job("J", line=99999)


# ── Bewegungsbefehle ──────────────────────────────────────────────────────────

def test_move_cartesian_valid_and_invalid_mode():
    c, sock = _client([_reply(hc._CMD_MOVE_CART)])
    m = CartesianMove(x=1000)
    c.move_cartesian(m, mode=2)
    req = _parse_req(sock.sent[0])
    assert req["cmd"] == hc._CMD_MOVE_CART and req["instance"] == 2
    assert req["payload"] == m.to_bytes()
    with pytest.raises(ValueError):
        c.move_cartesian(m, mode=9)


def test_move_pulse_valid_and_invalid_mode():
    c, sock = _client([_reply(hc._CMD_MOVE_PULSE)])
    m = PulseMove(robot_axes=[1, 2, 3, 4])
    c.move_pulse(m, mode=1)
    assert _parse_req(sock.sent[0])["payload"] == m.to_bytes()
    with pytest.raises(ValueError):
        c.move_pulse(m, mode=9)


# ════════════════════════════════════════════════════════════════════════════
# Plural / IO
# ════════════════════════════════════════════════════════════════════════════

def test_read_registers_plural():
    body = struct.pack("<I", 3) + struct.pack("<3H", 10, 20, 30)
    c, sock = _client([_reply(hc._CMD_REG_PLURAL, body)])
    assert c.read_registers(0, 3) == [10, 20, 30]
    req = _parse_req(sock.sent[0])
    assert req["service"] == hc._SVC_READ_ALL
    assert struct.unpack_from("<I", req["payload"], 0)[0] == 3


def test_write_registers_plural_payload_and_range():
    c, sock = _client([_reply(hc._CMD_REG_PLURAL)])
    c.write_registers(0, [1, 2])
    req = _parse_req(sock.sent[0])
    assert req["service"] == hc._SVC_WRITE_ALL
    assert struct.unpack_from("<I", req["payload"], 0)[0] == 2
    with pytest.raises(ValueError):
        c.write_registers(0, [70000])


def test_read_io_plural_rounds_odd_count_and_trims():
    body = struct.pack("<I", 2) + b"\xAB\xCD"
    c, sock = _client([_reply(hc._CMD_IO_PLURAL, body)])
    assert c.read_io_plural(10010, 1) == b"\xAB\xCD"   # count 1 -> aufgerundet auf 2
    assert struct.unpack_from("<I", _parse_req(sock.sent[0])["payload"], 0)[0] == 2


def test_write_io_uses_two_byte_payload():
    c, sock = _client([_reply(hc._CMD_IO_PLURAL)])
    c.write_io(27010, 0x5A)
    payload = _parse_req(sock.sent[0])["payload"]
    # 4-byte count(2) + 2 Datenbytes
    assert struct.unpack_from("<I", payload, 0)[0] == 2
    assert payload[4] == 0x5A and payload[5] == 0x00


# ════════════════════════════════════════════════════════════════════════════
# Telemetrie-Reads (Wrapper -> Decoder)
# ════════════════════════════════════════════════════════════════════════════

def test_read_status_decodes():
    c, _ = _client([_reply(hc._CMD_STATUS, struct.pack("<II", 0x05, 0x40))])
    st = c.read_status()
    assert st["step"] is True and st["auto"] is True and st["servo_on"] is True
    assert st["running"] is False


def test_read_status_too_short_raises():
    c, _ = _client([_reply(hc._CMD_STATUS, b"\x00\x00")])
    with pytest.raises(IOError):
        c.read_status()


def test_read_job_wrapper():
    data = b"JOBNAME".ljust(32, b"\x00") + struct.pack("<3I", 12, 34, 56)
    c, _ = _client([_reply(hc._CMD_JOB, data)])
    assert c.read_job(1) == {"job_name": "JOBNAME", "job_line": 12,
                             "job_step": 34, "job_speed": 56}


def test_read_axis_config_wrapper():
    data = b"S\x00\x00\x00L\x00\x00\x00U\x00\x00\x00R\x00\x00\x00" + b"\x00" * 16
    c, _ = _client([_reply(hc._CMD_AXIS_CFG, data)])
    cfg = c.read_axis_config(1)
    assert cfg["axes"] == ["S", "L", "U", "R"]


def test_read_converter_temp_on_yrc1000():
    c, _ = _client([_reply(hc._CMD_CONV_TEMP, struct.pack("<i", 37))])
    assert c.read_converter_temp(1) == 37


def test_read_encoder_temp_on_yrc1000():
    data = struct.pack("<8i", 30, 31, 32, 33, 0, 0, 0, 0)
    c, _ = _client([_reply(hc._CMD_ENC_TEMP, data)])
    out = c.read_encoder_temp(1)
    assert out["enc_temp_S"] == 30 and out["enc_temp_R"] == 33


def test_read_management_and_system_info():
    mgmt = b"2011/10/10 15:49" + b"000000:00'00"
    c, _ = _client([_reply(hc._CMD_MGMT_TIME, mgmt)])
    assert c.read_management_time(1)["start_time"] == "2011/10/10 15:49"

    sysinfo = b"YAS1.00".ljust(24, b"\x00") + b"YRC1000".ljust(16, b"\x00") + b"P1".ljust(8, b"\x00")
    c2, _ = _client([_reply(hc._CMD_SYS_INFO, sysinfo)])
    info = c2.read_system_info(11)
    assert info["sw_version"] == "YAS1.00" and info["model"] == "YRC1000"


# ── poll_all / _tx_multi ──────────────────────────────────────────────────────

def test_poll_all_status_and_job():
    status_data = struct.pack("<II", 0x40, 0x40)  # play + servo_on
    job_data = b"MASTER".ljust(32, b"\x00") + struct.pack("<3I", 1, 2, 3)
    c, _ = _client([
        _reply(hc._CMD_STATUS, status_data, req_id=1),
        _reply(hc._CMD_JOB, job_data, req_id=2),
    ])
    out = c.poll_all(include_job=True)
    assert out["servo_on"] is True
    assert out["job_name"] == "MASTER"


def test_poll_all_reads_every_variable_kind_in_one_burst():
    """D and R used to be reachable only as standalone reads, which put their
    value a round trip away from the rest of the row."""
    status_data = struct.pack("<II", 0x40, 0x40)
    c, _ = _client([
        _reply(hc._CMD_STATUS, status_data, req_id=1),
        _reply(hc._CMD_B_VAR, struct.pack("<B", 7), req_id=2),
        _reply(hc._CMD_I_VAR, struct.pack("<h", -3), req_id=3),
        _reply(hc._CMD_D_VAR, struct.pack("<i", -70000), req_id=4),
        _reply(hc._CMD_R_VAR, struct.pack("<f", 1.5), req_id=5),
    ])
    out = c.poll_all(
        include_job=False,
        b_vars=[{"label": "b", "no": 1}],
        i_vars=[{"label": "i", "no": 2}],
        d_vars=[{"label": "d", "no": 3}],
        r_vars=[{"label": "r", "no": 4}],
    )
    assert out["var_b"] == 7
    assert out["var_i"] == -3
    assert out["var_d"] == -70000          # signed 32-bit, beyond an I variable
    assert out["var_r"] == pytest.approx(1.5)


def test_poll_all_raises_on_status_error():
    c, _ = _client([_reply(hc._CMD_STATUS, b"", req_id=1, status=0x08)])
    with pytest.raises(HsesError):
        c.poll_all(include_job=False)


def test_tx_multi_marks_missing_response_none():
    c, _ = _client([_reply(hc._CMD_STATUS, b"ok", req_id=1)])
    reqs = [
        (hc._CMD_STATUS, 1, 0, hc._SVC_GET_ALL, b""),
        (hc._CMD_JOB, 1, 0, hc._SVC_GET_ALL, b""),
    ]
    results = c._tx_multi(reqs)
    assert results[0] is not None and results[0].data == b"ok"
    assert results[1] is None


# ════════════════════════════════════════════════════════════════════════════
# Lifecycle & Achsnamen
# ════════════════════════════════════════════════════════════════════════════

def test_context_manager_opens_and_closes(monkeypatch):
    created = FakeSocket()
    monkeypatch.setattr("yrcom.hses_client.socket.socket", lambda *a, **k: created)
    with HsesClient("10.0.0.9") as c:
        assert c._sock is created
    assert created.closed and c._sock is None


def test_open_binds_local_host(monkeypatch):
    created = FakeSocket()
    monkeypatch.setattr("yrcom.hses_client.socket.socket", lambda *a, **k: created)
    HsesClient("10.0.0.9", bind_host="10.0.0.100").open()
    assert created.bound == ("10.0.0.100", 0)


def test_load_axis_names_success_and_fallback():
    data = b"S\x00\x00\x00L\x00\x00\x00U\x00\x00\x00R\x00\x00\x00" + b"\x00" * 16
    c, _ = _client([_reply(hc._CMD_AXIS_CFG, data)])
    names = c.load_axis_names(1)
    assert names[:4] == ("S", "L", "U", "R")

    c2, _ = _client([])  # kein Reply -> read_axis_config wirft -> Fallback
    assert c2.load_axis_names(1) == _AXIS_NAMES


# ════════════════════════════════════════════════════════════════════════════
# Reine Decoder
# ════════════════════════════════════════════════════════════════════════════

def test_decode_status_bits_and_model_key():
    st = _decode_status(struct.pack("<II", 0x10, 0x00), MODEL_YRC1000)
    assert st["speed_limit"] is True
    st_dx = _decode_status(struct.pack("<II", 0x10, 0x00), MODEL_DX100)
    assert st_dx["in_guard_safe"] is True
    assert _decode_status(b"\x00") == {}


def test_decode_alarm_minimal_and_full():
    assert _decode_alarm(b"\x00" * 4)["alarm_code"] == 0
    data = struct.pack("<3I", 1000, 2, 1) + b"2011/01/01 00:00".ljust(16, b"\x00") \
        + b"OVERRUN".ljust(32, b"\x00")
    out = _decode_alarm(data)
    assert out["alarm_code"] == 1000 and out["alarm_type"] == 1
    assert out["alarm_name"] == "OVERRUN"


def test_decode_alarm_sub_adds_strings():
    base = struct.pack("<3I", 5, 0, 1) + b"\x00" * 16 + b"\x00" * 32
    data = base + b"[SV#1]".ljust(16, b"\x00") + b"detail".ljust(96, b"\x00") + b"0".ljust(96, b"\x00")
    out = _decode_alarm_sub(data)
    assert out["sub_code_info"] == "[SV#1]" and out["sub_code_str"] == "detail"


def test_decode_job_pure():
    data = b"J".ljust(32, b"\x00") + struct.pack("<3I", 7, 8, 9)
    assert _decode_job(data) == {"job_name": "J", "job_line": 7, "job_step": 8, "job_speed": 9}


def test_cstr_strips():
    assert _cstr(b"[SV#1]\x00\x00garbage") == "[SV#1]"
    assert _cstr(b"  spaced  \x00") == "spaced"


def test_decode_regen_power():
    data = struct.pack("<I", 0b1) + bytes([1, 0, 0, 0]) + struct.pack("<3i", 10, 20, 30)
    out = _decode_regen_power(data)
    assert out["valid_boards"] == 1 and out["status"] == 1
    assert out["boards"][0] == {"board": 1, "valid": True,
                                "today": 10, "yesterday": 20, "cumulative": 30}


def test_decode_management_time():
    data = b"2011/10/10 15:49" + b"000000:00'00"
    out = _decode_management_time(data)
    assert out["start_time"] == "2011/10/10 15:49"
    assert out["elapse_time"] == "000000:00'00"


def test_decode_system_info():
    data = b"YAS1.00".ljust(24, b"\x00") + b"YRC1000".ljust(16, b"\x00") + b"P1".ljust(8, b"\x00")
    out = _decode_system_info(data)
    assert out == {"sw_version": "YAS1.00", "model": "YRC1000", "parameter_version": "P1"}


def test_decode_axis_config_lists_present_axes():
    data = b"S\x00\x00\x00L\x00\x00\x00" + b"\x00" * 24
    out = _decode_axis_config(data)
    assert out["axis_1"] == "S" and out["axes"] == ["S", "L"]


def test_decode_axis_int32():
    data = struct.pack("<8i", 1, -2, 3, -4, 0, 0, 0, 0)
    out = _decode_axis_int32(data, "pe")
    assert out["pe_S"] == 1 and out["pe_L"] == -2 and out["pe_R"] == -4


def test_decode_position_pulse_and_cartesian():
    pulse = struct.pack("<13i", 0, 0, 0, 0, 0, 100, 200, 300, 400, 0, 0, 0, 0)
    out = _decode_position(pulse)
    assert out["pos_S"] == 100 and out["pos_R"] == 400

    cart = struct.pack("<13i", 16, 0, 0, 0, 0, 1000, 2000, 3000, 450000, 0, 0, 0, 0)
    outc = _decode_position(cart)
    assert outc["pos_X"] == 1.0 and outc["pos_Rx"] == 45.0


# ════════════════════════════════════════════════════════════════════════════
# Scalar Short-Data-Zweige (fehlende Daten -> 0)
# ════════════════════════════════════════════════════════════════════════════

def test_scalar_reads_return_zero_on_short_data():
    c, _ = _client([_reply(hc._CMD_B_VAR, b"")])
    assert c.read_b_var(1) == 0
    c, _ = _client([_reply(hc._CMD_I_VAR, b"\x00")])
    assert c.read_i_var(1) == 0
    c, _ = _client([_reply(hc._CMD_D_VAR, b"\x00\x00")])
    assert c.read_d_var(1) == 0
    c, _ = _client([_reply(hc._CMD_R_VAR, b"\x00")])
    assert c.read_r_var(1) == 0.0
    c, _ = _client([_reply(hc._CMD_IO, b"")])
    assert c.read_io(10010) == 0


# ════════════════════════════════════════════════════════════════════════════
# Restliche Telemetrie-Wrapper
# ════════════════════════════════════════════════════════════════════════════

def test_read_alarm_variants():
    data = struct.pack("<3I", 4107, 0, 2)
    for meth, cmd in [("read_alarm", hc._CMD_ALARM),
                      ("read_alarm_history", hc._CMD_ALARM_HIST)]:
        c, sock = _client([_reply(cmd, data)])
        out = getattr(c, meth)(1)
        assert out["alarm_code"] == 4107 and out["alarm_type"] == 2
        assert _parse_req(sock.sent[0])["cmd"] == cmd


def test_read_alarm_sub_variants():
    base = struct.pack("<3I", 9, 0, 1) + b"\x00" * 48
    data = base + b"[SV#1]".ljust(16, b"\x00") + b"x".ljust(96, b"\x00") + b"0".ljust(96, b"\x00")
    for meth, cmd in [("read_alarm_sub", hc._CMD_ALARM_SUB),
                      ("read_alarm_history_sub", hc._CMD_ALARM_HIST_SUB)]:
        c, _ = _client([_reply(cmd, data)])
        out = getattr(c, meth)(1)
        assert out["sub_code_info"] == "[SV#1]"


def test_read_position_error_and_torque():
    data = struct.pack("<8i", 5, 6, 7, 8, 0, 0, 0, 0)
    c, _ = _client([_reply(hc._CMD_POSERR, data)])
    assert c.read_position_error(1)["pe_S"] == 5
    c, _ = _client([_reply(hc._CMD_TORQUE, data)])
    assert c.read_torque(1)["tq_L"] == 6


def test_read_position_wrapper():
    pulse = struct.pack("<13i", 0, 0, 0, 0, 0, 100, 200, 300, 400, 0, 0, 0, 0)
    c, _ = _client([_reply(hc._CMD_POSITION, pulse)])
    assert c.read_position(1)["pos_S"] == 100


def test_read_regen_power_on_yrc1000():
    data = struct.pack("<I", 1) + bytes([2, 1, 0, 0]) + struct.pack("<3i", 1, 2, 3)
    c, _ = _client([_reply(hc._CMD_REGEN_PWR, data)])
    out = c.read_regen_power()
    assert out["status"] == 2 and out["unit"] == 1


# ════════════════════════════════════════════════════════════════════════════
# Plural Variable Wrapper
# ════════════════════════════════════════════════════════════════════════════

def _plural_body(count: int, packed: bytes) -> bytes:
    return struct.pack("<I", count) + packed


def test_read_and_write_b_vars():
    c, sock = _client([
        _reply(hc._CMD_B_PLURAL, _plural_body(2, bytes([10, 20]))),
        _reply(hc._CMD_B_PLURAL),
    ])
    assert c.read_b_vars(0, 2) == [10, 20]
    c.write_b_vars(0, [1, 2])
    assert _parse_req(sock.sent[1])["payload"][4:] == bytes([1, 2])


def test_write_b_vars_rejects_odd_count():
    c, _ = _client([_reply(hc._CMD_B_PLURAL)])
    with pytest.raises(ValueError):
        c.write_b_vars(0, [1, 2, 3])


def test_read_and_write_i_vars():
    c, sock = _client([
        _reply(hc._CMD_I_PLURAL, _plural_body(2, struct.pack("<2h", -1, 2))),
        _reply(hc._CMD_I_PLURAL),
    ])
    assert c.read_i_vars(0, 2) == [-1, 2]
    c.write_i_vars(0, [3, 4])
    with pytest.raises(ValueError):
        c.write_i_vars(0, [99999])


def test_read_and_write_d_vars():
    c, sock = _client([
        _reply(hc._CMD_D_PLURAL, _plural_body(2, struct.pack("<2i", -100, 200))),
        _reply(hc._CMD_D_PLURAL),
    ])
    assert c.read_d_vars(0, 2) == [-100, 200]
    c.write_d_vars(0, [1, 2])
    with pytest.raises(ValueError):
        c.write_d_vars(0, [9_000_000_000])


def test_read_and_write_r_vars():
    c, sock = _client([
        _reply(hc._CMD_R_PLURAL, _plural_body(2, struct.pack("<2f", 1.5, 2.5))),
        _reply(hc._CMD_R_PLURAL),
    ])
    assert c.read_r_vars(0, 2) == [1.5, 2.5]
    c.write_r_vars(0, [3.5])
    assert struct.unpack_from("<f", _parse_req(sock.sent[1])["payload"], 4)[0] == 3.5


def test_read_and_write_s_vars():
    entry = b"HI".ljust(32, b"\x00")
    c, sock = _client([
        _reply(hc._CMD_S_PLURAL, _plural_body(1, entry)),
        _reply(hc._CMD_S_PLURAL),
    ])
    assert c.read_s_vars(0, 1) == ["HI"]
    c.write_s_vars(0, ["AB"])
    payload = _parse_req(sock.sent[1])["payload"][4:]
    assert len(payload) == 32 and payload[:2] == b"AB"


def test_read_and_write_position_vars():
    p = PositionVar(data_type=17, axes=[1, 2, 3, 4, 5, 6, 7, 8])
    c, sock = _client([
        _reply(hc._CMD_P_PLURAL, _plural_body(1, p.to_bytes())),
        _reply(hc._CMD_P_PLURAL),
    ])
    assert c.read_position_vars(0, 1) == [p]
    c.write_position_vars(0, [p])
    assert _parse_req(sock.sent[1])["payload"][4:] == p.to_bytes()


def test_read_and_write_bp_and_ex_vars():
    a = AxisPositionVar(data_type=0, axes=[1, 2, 3, 4, 5, 6, 7, 8])
    c, sock = _client([
        _reply(hc._CMD_BP_PLURAL, _plural_body(1, a.to_bytes())),
        _reply(hc._CMD_BP_PLURAL),
        _reply(hc._CMD_EX_PLURAL, _plural_body(1, a.to_bytes())),
        _reply(hc._CMD_EX_PLURAL),
    ])
    assert c.read_bp_vars(0, 1) == [a]
    c.write_bp_vars(0, [a])
    assert c.read_ex_vars(0, 1) == [a]
    c.write_ex_vars(0, [a])
    assert _parse_req(sock.sent[3])["payload"][4:] == a.to_bytes()


def test_write_ex_var_scalar_payload():
    a = AxisPositionVar(data_type=0, axes=[9, 8, 7, 6, 5, 4, 3, 2])
    c, sock = _client([_reply(hc._CMD_EX_VAR)])
    c.write_ex_var(1, a)
    assert _parse_req(sock.sent[0])["payload"] == a.to_bytes()


# ── IO-Plural Randfaelle ──────────────────────────────────────────────────────

def test_read_io_plural_short_data_returns_empty():
    c, _ = _client([_reply(hc._CMD_IO_PLURAL, b"\x00")])
    assert c.read_io_plural(10010, 2) == b""


def test_write_io_plural_pads_odd_data():
    c, sock = _client([_reply(hc._CMD_IO_PLURAL)])
    c.write_io_plural(27010, b"\x01")  # ungerade -> auf 2 aufgefuellt
    payload = _parse_req(sock.sent[0])["payload"]
    assert struct.unpack_from("<I", payload, 0)[0] == 2
    assert payload[4:] == b"\x01\x00"


# ════════════════════════════════════════════════════════════════════════════
# poll_all mit voller Konfiguration
# ════════════════════════════════════════════════════════════════════════════

def test_poll_all_full_config():
    status_data = struct.pack("<II", 0x40, 0x40)
    alarm_data = struct.pack("<3I", 100, 0, 1)
    pos_data = struct.pack("<13i", 0, 0, 0, 0, 0, 100, 200, 300, 400, 0, 0, 0, 0)
    cio_data = struct.pack("<I", 2) + bytes([0x34, 0x12])
    io_data = struct.pack("<I", 2) + bytes([0x99, 0x00])
    # Reihenfolge/IDs wie poll_all sie vergibt: status,alarm1,position,b_var,i_var,cio,io_plural
    c, _ = _client([
        _reply(hc._CMD_STATUS, status_data, req_id=1),
        _reply(hc._CMD_ALARM, alarm_data, req_id=2),
        _reply(hc._CMD_POSITION, pos_data, req_id=3),
        _reply(hc._CMD_B_VAR, bytes([7]), req_id=4),
        _reply(hc._CMD_I_VAR, struct.pack("<h", -3), req_id=5),
        _reply(hc._CMD_IO_PLURAL, cio_data, req_id=6),
        _reply(hc._CMD_IO_PLURAL, io_data, req_id=7),
    ])
    out = c.poll_all(
        alarm_slots=1, include_job=False, position_group=1,
        b_vars=[{"label": "x", "no": 0}], i_vars=[{"label": "y", "no": 0}],
        io_signals=[{"label": "z", "logical": 10010}], read_cio=True,
    )
    assert out["servo_on"] is True
    assert out["alarm1_type"] == 1
    assert out["pos_S"] == 100
    assert out["var_x"] == 7
    assert out["var_y"] == -3
    assert out["cio_counter"] == 0x1234
    assert out["io_z"] == 0x99


# ════════════════════════════════════════════════════════════════════════════
# Decoder Short-Data-Zweige
# ════════════════════════════════════════════════════════════════════════════

def test_decoders_handle_short_data():
    assert _decode_job(b"short")["job_name"] == ""
    assert _decode_regen_power(b"\x00\x00\x00\x00")["boards"] == []
    assert _decode_axis_int32(b"", "x") == {}
    assert _decode_position(b"\x00" * 8) == {}


# ════════════════════════════════════════════════════════════════════════════
# Restliche Randfaelle (Lifecycle, _tx added, _tx_multi, Write-Erfolgspfade)
# ════════════════════════════════════════════════════════════════════════════

def test_open_is_idempotent_when_socket_present():
    c, sock = _client()
    c.open()  # _sock schon gesetzt -> No-op
    assert c._sock is sock


def test_close_swallows_socket_error():
    c = HsesClient("10.0.0.9")
    c._sock = FakeSocket(close_raises=True)
    c.close()
    assert c._sock is None


def test_tx_parses_added_status():
    c, _ = _client([_reply(hc._CMD_STATUS, b"x", added=0x1234, added_size=1)])
    r = c._tx(hc._CMD_STATUS)
    assert r.added == 0x1234


def test_tx_multi_requires_open():
    c = HsesClient("10.0.0.9")
    with pytest.raises(RuntimeError):
        c._tx_multi([(hc._CMD_STATUS, 1, 0, hc._SVC_GET_ALL, b"")])


def test_tx_multi_deadline_break_returns_none():
    c = HsesClient("10.0.0.9", timeout=0)
    c._sock = FakeSocket([])
    assert c._tx_multi([(hc._CMD_STATUS, 1, 0, hc._SVC_GET_ALL, b"")]) == [None]


def test_tx_multi_ignores_malformed_reply():
    c, _ = _client([b"short"])
    assert c._tx_multi([(hc._CMD_STATUS, 1, 0, hc._SVC_GET_ALL, b"")]) == [None]


def test_tx_multi_ignores_unknown_req_id():
    # echo_req_id=False: the foreign ID 99 must survive, otherwise the test
    # no longer proves anything.
    c, _ = _client([_reply(hc._CMD_STATUS, b"x", req_id=99)], echo_req_id=False)
    assert c._tx_multi([(hc._CMD_STATUS, 1, 0, hc._SVC_GET_ALL, b"")]) == [None]


def test_poll_all_skips_missing_response():
    c, _ = _client([_reply(hc._CMD_STATUS, struct.pack("<II", 0x40, 0x40), req_id=1)])
    out = c.poll_all(include_job=True)  # job requested (id 2) but no reply
    assert out["servo_on"] is True
    assert "job_name" not in out


def test_scalar_write_success_payloads():
    c, sock = _client([_reply(hc._CMD_B_VAR), _reply(hc._CMD_I_VAR), _reply(hc._CMD_D_VAR)])
    c.write_b_var(1, 42)
    c.write_i_var(1, -5)
    c.write_d_var(1, -100000)
    assert struct.unpack_from("<I", _parse_req(sock.sent[0])["payload"], 0)[0] == 42
    assert struct.unpack_from("<i", _parse_req(sock.sent[1])["payload"], 0)[0] == -5
    assert struct.unpack_from("<i", _parse_req(sock.sent[2])["payload"], 0)[0] == -100000


def test_read_plural_short_data_returns_empty():
    c, _ = _client([_reply(hc._CMD_REG_PLURAL, b"\x00")])
    assert c.read_registers(0, 1) == []


def test_write_b_vars_rejects_out_of_range_value():
    c, _ = _client([_reply(hc._CMD_B_PLURAL)])
    with pytest.raises(ValueError):
        c.write_b_vars(0, [10, 300])


def test_read_converter_temp_short_data_returns_zero():
    c, _ = _client([_reply(hc._CMD_CONV_TEMP, b"\x00\x00")])
    assert c.read_converter_temp(1) == 0


# ── Verspaetete Antworten nach einem Timeout (Response-Versatz) ───────────────

def test_tx_discards_stale_reply_from_earlier_request():
    """After a timeout the late reply is still in the socket buffer and would
    otherwise be read as the reply to the NEXT request — a persistent offset
    where e.g. job-name bytes arrive as a B variable. The reply must be
    matched by request ID (as in _tx_multi)."""
    c, sock = _client(echo_req_id=False)
    c._req_id = 5
    # Buffer: first the late reply to id=4, then the real one for id=5.
    sock.replies = [
        _reply(0x73, b"STALE-JOB-NAME", req_id=4),
        _reply(0x72, struct.pack("<II", 0x40, 0x40), req_id=5),
    ]
    resp = c._tx(0x72)
    assert resp.cmd == 0x72
    assert resp.data == struct.pack("<II", 0x40, 0x40)


def test_tx_times_out_when_only_stale_replies_arrive():
    """Nur fremde Antworten => sauberer Timeout, kein falscher Treffer."""
    c, sock = _client(echo_req_id=False)
    c._req_id = 9
    sock.replies = [_reply(0x73, b"X", req_id=1), _reply(0x73, b"Y", req_id=2)]
    with pytest.raises(socket.timeout):
        c._tx(0x72)
