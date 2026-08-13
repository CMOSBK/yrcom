"""
Server-side HSES codec — the mirror image of hses_client.py's request
builder/response parser. Decodes incoming request bytes into a
ServerCommand; encodes results back into wire bytes.

Wire header is the 32-byte YERC header, identical in both directions:
    0:4    magic "YERC"
    4:6    header size (u16, always 32)
    6:8    data size (u16)
    8      Reserve1 (0x03 fixed)
    9      processing division (1 = robot control)
    10     ACK (0 = request, 1 = response)
    11     request id (echoed back)
    12:16  block number
    16:24  Reserve2 ("99999999" fixed)
    24:26  command (u16) — in a response this is the ECHOED command
    26:28  request: instance (u16)   | response: status (byte) + pad
    28     request: attribute (byte) | response: added-status size (byte)
    29     request: service (byte)   | response: pad
    30:32  request: padding          | response: added-status word (u16)
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

_MAGIC = b"YERC"
_HDR_SIZE = 0x0020


@dataclass(frozen=True)
class ServerCommand:
    cmd: int
    instance: int
    attribute: int
    service: int
    req_id: int
    payload: bytes


class MalformedRequest(Exception):
    """Raw bytes are too short, or the header magic/size doesn't match."""


def decode_request(raw: bytes) -> ServerCommand:
    if len(raw) < _HDR_SIZE:
        raise MalformedRequest(f"request too short: {len(raw)} B")
    if raw[0:4] != _MAGIC:
        raise MalformedRequest(f"bad magic: {raw[0:4]!r}")
    data_size = struct.unpack_from("<H", raw, 6)[0]
    req_id = raw[11]
    cmd = struct.unpack_from("<H", raw, 24)[0]
    instance = struct.unpack_from("<H", raw, 26)[0]
    attribute = raw[28]
    service = raw[29]
    payload = raw[_HDR_SIZE:_HDR_SIZE + data_size]
    return ServerCommand(cmd=cmd, instance=instance, attribute=attribute,
                          service=service, req_id=req_id, payload=payload)


def _response_header(cmd: int, req_id: int, data_size: int) -> bytearray:
    hdr = bytearray(_HDR_SIZE)
    hdr[0:4] = _MAGIC
    struct.pack_into("<H", hdr, 4, _HDR_SIZE)
    struct.pack_into("<H", hdr, 6, data_size)
    hdr[8] = 0x03
    hdr[9] = 1          # processing division: robot control
    hdr[10] = 0x01       # ACK = response
    hdr[11] = req_id
    struct.pack_into("<I", hdr, 12, 0)
    hdr[16:24] = b"99999999"
    struct.pack_into("<H", hdr, 24, cmd)
    return hdr


def encode_response(cmd: int, req_id: int, data: bytes = b"") -> bytes:
    """Success response (status 0x00, no added-status)."""
    hdr = _response_header(cmd, req_id, len(data))
    hdr[26] = 0x00   # status
    hdr[27] = 0
    hdr[28] = 0       # added-status size
    struct.pack_into("<H", hdr, 30, 0)
    return bytes(hdr) + data


def encode_error(cmd: int, req_id: int, status: int, added: int = 0) -> bytes:
    """Error response — empty data, given status/added-status."""
    hdr = _response_header(cmd, req_id, 0)
    hdr[26] = status
    hdr[27] = 0
    hdr[28] = 1 if added else 0
    struct.pack_into("<H", hdr, 30, added)
    return bytes(hdr)
