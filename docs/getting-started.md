# Getting started

## Installation

```bash
pip install yrcom           # from PyPI
```

For development (editable install incl. test tooling):

```bash
pip install -e ".[dev]"
```

`yrcom` is pure stdlib — it pulls in **no runtime dependencies**.
Python 3.11 or newer is required.

## First call over HSES

`HsesClient` speaks the High Speed Ethernet Server protocol (UDP 10040) — the
fast path for status, variables, I/O and telemetry.

```python
from yrcom import HsesClient

c = HsesClient("192.0.2.10")    # UDP 10040
c.open()
print(c.read_status())          # {'servo_on': False, 'teach': True, ...}
print(c.read_d_var(0))          # D[0]
c.close()
```

`HsesClient` is also a context manager, so `open()`/`close()` can be implicit:

```python
from yrcom import HsesClient

with HsesClient("192.0.2.10") as c:
    print(c.read_alarms())
```

## Transport-agnostic via RobotConnection

When you want one object that can talk over **either** HSES or the classic
Ethernet Server, use [`RobotConnection`](reference/connection.md) and pick the
transport by name — the high-level methods return the same keys either way.

```python
from yrcom import RobotConnection

rc = RobotConnection("192.0.2.10", transport="hses")
rc.connect()
print(rc.status())              # same keys, whether HSES or ES
rc.disconnect()
```

## Networking checklist

- The controller's robot IP must be reachable and your interface must sit in
  the same subnet.
- HSES uses **UDP 10040** (robot control) and **UDP 10041** (file control);
  the Ethernet Server uses **TCP 80**. Make sure a firewall is not dropping them.
- No robot on the bench? Jump straight to [Offline testing](offline-testing.md).
