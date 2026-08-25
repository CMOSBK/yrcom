# Transports

A Yaskawa controller exposes several communication channels. `yrcom` wraps each
one in its own client so you can reach for the right tool, and bundles them
behind a single object when you don't want to care which is which.

## Which client do I want?

| I want to… | Use | Transport |
|---|---|---|
| read status, alarms, variables, I/O, telemetry, drive motion — fast | [`HsesClient`](reference/hses-client.md) | HSES, UDP 10040 |
| use the classic host-control command set | [`YaskawaClient`](reference/ethernet-server.md) | Ethernet Server, TCP 80 |
| read/write job files, list files, pull a CMOS backup | [`HsesFileClient`](reference/file-client.md) | HSES File Control, UDP 10041 |
| not care which transport, one object, same methods | [`RobotConnection`](reference/connection.md) | ES + HSES behind one IP |
| develop and test without a robot | [`HsesReplayServer`](reference/simulator.md) | replayed capture |

## HSES vs. Ethernet Server

**HSES** (High Speed Ethernet Server) is the modern, UDP-based protocol. It is
the fast path and carries the widest command set — 45 robot-control commands
in `yrcom`, all verified against real hardware.

The **Ethernet Server** is the older TCP host-control interface. It is still
useful for the classic command set and for controllers or setups where HSES is
not available.

**File Control** is a separate HSES service on **UDP 10041** for reading and
writing files on the controller (job files, the file list, a full CMOS backup).

## RobotConnection: one IP, pick the transport

`RobotConnection` bundles the Ethernet Server and HSES behind one address and
lets you select the transport by name:

```python
from yrcom import RobotConnection, TRANSPORT_HSES, TRANSPORT_ETHERNET

rc = RobotConnection("192.0.2.10", transport=TRANSPORT_HSES)
rc.connect()
print(rc.status())
rc.disconnect()
```

Its high-level helpers (`status()`, variable and I/O access, …) return the same
shape regardless of the underlying transport, so calling code stays the same
whether it runs over HSES or ES.

### A deliberately blocked write path

Switching a **single I/O output over HSES** raises
[`HsesWriteBlockedError`](reference/connection.md) on purpose. The HSES output
write requires an even byte count, so a single byte would be padded with a hard
zero — silently clearing the neighbouring output byte (real actuators in the
cell). Rather than compensate on speculation, `yrcom` blocks that path. Reading
I/O is unaffected.
