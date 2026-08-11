"""
Yaskawa YRC1000 — HSES File Control client.

A separate transport from the telemetry client (`hses_client.HsesClient`):
different UDP port (10041), Processing Division 2, and a multi-packet Data/ACK
handshake instead of a single request/response. The sub-header carries a fixed
Command/Instance/Attribute of 0 — the operation is chosen by the Service byte
alone (0x09 delete, 0x15 load, 0x16 save, 0x32 list). See manual HW1483358
(File Control Function).

File transfers are chunked into <=479-byte data parts. Each data packet's Block
No. increments; the final packet is flagged with bit 0x8000_0000. The receiving
side acknowledges every data packet with an empty ACK (echoing the block number)
before the next one is sent.

Command Remote (RS 005 = 1) is required for write/delete/transfer access.
"""
from __future__ import annotations

import logging
import struct
from typing import Optional

from .hses_client import _MAGIC, _HDR_SIZE, HsesError, _HsesSocket

# Diagnostic trace of the file-control layer (Roadmap I convention); the library
# only emits, the app configures handlers. Requests/ACKs → DEBUG, file operations
# → INFO, robot errors / malformed replies → ERROR.
log = logging.getLogger(__name__)

HSES_FILE_PORT = 10041
_PROC_FILE     = 2         # processing division: file control

# Service bytes (the sub-header Command/Instance/Attribute are all 0)
_SVC_DELETE = 0x09         # File Deleting        (no transfer)
_SVC_LOAD   = 0x15         # File Loading         PC  -> YRC (write a file)
_SVC_SAVE   = 0x16         # File Saving          YRC -> PC (read a file)
_SVC_LIST   = 0x32         # File List            YRC -> PC

_ACK_REQUEST = 0x00        # header offset 10: this packet is a request
_ACK_ANSWER  = 0x01        # this packet is an answer / acknowledgement
_LAST_BLOCK  = 0x8000_0000 # block-no bit marking the final packet
_MAX_DATA    = 479         # max data-part size per packet

_CMOS_PATH = "/SPDRV/CMOSBK.BIN"   # batch CMOS backup (Device = RAMDISK, ~10 min)


class HsesFileClient(_HsesSocket):
    """
    UDP client for the Yaskawa YRC1000 HSES File Control port (10041).

    Usage:
        with HsesFileClient("192.0.2.10") as f:
            for name in f.list_files("*.JBI"):
                print(name)
            data = f.read_file("TESTJOB.JBI")
    """

    def __init__(self, host: str, port: int = HSES_FILE_PORT, timeout: float = 5.0,
                 bind_host: Optional[str] = None) -> None:
        super().__init__(host, port, timeout, bind_host)

    # ── packet building / parsing ─────────────────────────────────────────────

    @staticmethod
    def _build(service: int, req_id: int, block_no: int, ack: int,
               payload: bytes = b"") -> bytes:
        """Build a 32-byte File Control header (Command/Instance/Attribute = 0)."""
        hdr = bytearray(_HDR_SIZE)
        hdr[0:4] = _MAGIC
        struct.pack_into("<H", hdr,  4, _HDR_SIZE)
        struct.pack_into("<H", hdr,  6, len(payload))
        hdr[8]  = 0x03                                  # Reserve1  FIXED
        hdr[9]  = _PROC_FILE
        hdr[10] = ack
        hdr[11] = req_id
        struct.pack_into("<I", hdr, 12, block_no & 0xFFFF_FFFF)
        hdr[16:24] = b"99999999"                        # Reserve2  FIXED
        struct.pack_into("<H", hdr, 24, 0)             # Command No. = 0
        struct.pack_into("<H", hdr, 26, 0)             # Instance    = 0
        hdr[28] = 0                                     # Attribute   = 0
        hdr[29] = service
        struct.pack_into("<H", hdr, 30, 0)             # Padding
        return bytes(hdr) + payload

    def _recv(self, req_service: int) -> tuple:
        """
        Receive one packet and parse it. Returns (block_no, data, added).
        Raises HsesError on a non-zero status, IOError on a malformed reply.
        The answer's status/added fields sit at the same offsets as the robot
        control response (status raw[26], added-size raw[28], added raw[30]).
        """
        raw = self._sock.recv(4096)
        if len(raw) < _HDR_SIZE:
            log.error("malformed file reply svc=0x%02X: too short (%d B)",
                      req_service, len(raw))
            raise IOError(f"Response too short: {len(raw)} B")
        if raw[0:4] != _MAGIC:
            log.error("malformed file reply svc=0x%02X: bad magic %r",
                      req_service, raw[0:4])
            raise IOError(f"Bad magic: {raw[0:4]!r}")

        dat_size    = struct.unpack_from("<H", raw,  6)[0]
        block_no    = struct.unpack_from("<I", raw, 12)[0]
        resp_status = raw[26]
        added_size  = raw[28]
        added       = struct.unpack_from("<H", raw, 30)[0] if added_size >= 1 else 0
        data        = raw[_HDR_SIZE: _HDR_SIZE + dat_size]

        if resp_status != 0x00:
            log.error("file reply svc=0x%02X status=0x%02X added=0x%04X",
                      req_service, resp_status, added)
            raise HsesError(_PROC_FILE, resp_status, added)

        return block_no, data, added

    # ── receive transfer (File List 0x32, File Saving 0x16) ────────────────────

    def _read_transfer(self, service: int, request_payload: bytes) -> bytes:
        """
        Run a YRC → PC transfer: send the request, then receive data packets,
        acknowledging each, until the packet flagged with _LAST_BLOCK arrives.
        Returns the concatenated data parts.

        A status-0 reply carrying an added status (e.g. 0xE2A7 file not found,
        0xE29C size 0) is treated as an empty result, not an error.
        """
        if self._sock is None:
            raise RuntimeError("Call open() first")
        rid = self._bump_req_id()
        log.debug("file tx svc=0x%02X id=%d payload=%dB",
                  service, rid, len(request_payload))
        self._sock.sendto(self._build(service, rid, 0, _ACK_REQUEST, request_payload),
                          (self._host, self._port))

        chunks: list[bytes] = []
        while True:
            block_no, data, added = self._recv(service)
            if added and not data:
                log.warning("file svc=0x%02X id=%d empty (added=0x%04X)",
                            service, rid, added)
                return b""
            chunks.append(data)
            # Acknowledge this packet (empty data part, echo the block number).
            self._sock.sendto(self._build(service, rid, block_no, _ACK_ANSWER),
                              (self._host, self._port))
            if block_no & _LAST_BLOCK:
                break
        out = b"".join(chunks)
        log.debug("file rx svc=0x%02X id=%d total=%dB", service, rid, len(out))
        return out

    # ── send transfer (File Loading 0x15) ──────────────────────────────────────

    def _write_transfer(self, service: int, request_payload: bytes,
                        content: bytes) -> None:
        """
        Run a PC → YRC transfer: send the request, wait for the robot's ACK, then
        send `content` as _MAX_DATA-sized data packets, waiting for an ACK after
        each. The final packet is flagged with _LAST_BLOCK.
        """
        if self._sock is None:
            raise RuntimeError("Call open() first")
        rid = self._bump_req_id()
        log.debug("file tx svc=0x%02X id=%d payload=%dB content=%dB",
                  service, rid, len(request_payload), len(content))
        self._sock.sendto(self._build(service, rid, 0, _ACK_REQUEST, request_payload),
                          (self._host, self._port))
        self._recv(service)   # robot acknowledges the request (raises on error)

        parts = [content[i:i + _MAX_DATA] for i in range(0, len(content), _MAX_DATA)] or [b""]
        for i, part in enumerate(parts):
            last = i == len(parts) - 1
            block_no = (i + 1) | (_LAST_BLOCK if last else 0)
            self._sock.sendto(self._build(service, rid, block_no, _ACK_ANSWER, part),
                              (self._host, self._port))
            self._recv(service)   # robot acknowledges each data packet
        log.debug("file tx svc=0x%02X id=%d done (%d packets)", service, rid, len(parts))

    # ── public operations ──────────────────────────────────────────────────────

    def list_files(self, pattern: str = "*.JBI") -> "list[str]":
        """
        File List (service 0x32). `pattern` filters by extension — e.g. *.JBI,
        *.DAT, *.CND, *.PRM, *.SYS, *.LST; an empty/`*.*` filter lists jobs.
        Returns the file names as a list (blank lines dropped).
        """
        log.info("file: list %r", pattern)
        raw = self._read_transfer(_SVC_LIST, pattern.encode("ascii", errors="replace"))
        text = raw.decode("latin-1", errors="replace")
        return [line.strip() for line in text.split("\r\n") if line.strip()]

    def read_file(self, name: str) -> bytes:
        """
        File Saving (service 0x16) — read a file from the controller (YRC → PC).
        `name` is the file name as shown on the pendant (e.g. "TESTJOB.JBI").
        Returns the raw file bytes (ASCII content with <CR><LF> line breaks).
        """
        log.info("file: read %r", name)
        return self._read_transfer(_SVC_SAVE, name.encode("ascii", errors="replace"))

    def write_file(self, name: str, content: bytes) -> None:
        """
        File Loading (service 0x15) — send a file to the controller (PC → YRC).
        `name` is the target file name; `content` the raw file bytes (ASCII,
        <CR><LF> line breaks). Requires Command Remote (RS 005 = 1).
        """
        log.info("file: write %r (%d B)", name, len(content))
        self._write_transfer(_SVC_LOAD, name.encode("ascii", errors="replace"), content)

    def delete_file(self, name: str) -> None:
        """
        File Deleting (service 0x09). Deletes `name` on the controller — a single
        request/response, no transfer. Requires Command Remote (RS 005 = 1).
        """
        if self._sock is None:
            raise RuntimeError("Call open() first")
        log.info("file: delete %r", name)
        rid = self._bump_req_id()
        self._sock.sendto(
            self._build(_SVC_DELETE, rid, 0, _ACK_REQUEST,
                        name.encode("ascii", errors="replace")),
            (self._host, self._port),
        )
        self._recv(_SVC_DELETE)   # raises HsesError on a non-zero status

    def backup_cmos(self) -> bytes:
        """
        Batch CMOS backup — read the complete CMOSBK.BIN via File Saving (0x16).
        The controller's Device must be RAMDISK and the transfer takes ~10 min,
        so use a generous timeout. Returns the raw backup bytes.
        """
        log.info("file: CMOS batch backup %s", _CMOS_PATH)
        return self._read_transfer(_SVC_SAVE, _CMOS_PATH.encode("ascii"))
