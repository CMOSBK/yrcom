# yrcom

[![CI](https://github.com/CMOSBK/yrcom/actions/workflows/ci.yml/badge.svg)](https://github.com/CMOSBK/yrcom/actions/workflows/ci.yml)

📖 **Documentation: <https://cmosbk.github.io/yrcom/>** — getting started, the
transport guide and the full API reference (every command, generated from the
source).

Yaskawa robot communication as a standalone Python library — a single,
hardware-verified communication core meant to be shared across projects
instead of being duplicated.

The name: **YRC** (Yaskawa Robot Controller, e.g. YRC1000) + **COM**munication.

**Pure stdlib** — no runtime dependencies. Import name: `yrcom`.

## Transports

| Class | Transport | Purpose |
|---|---|---|
| `YaskawaClient` | Ethernet Server, TCP 80 | classic host-control commands |
| `HsesClient` | HSES, UDP 10040 | high speed: 45 commands (status, alarms, variables, IO, telemetry, motion) |
| `HsesFileClient` | HSES File Control, UDP 10041 | read/write job files, file list, CMOS backup |
| `RobotConnection` | ES + HSES behind one IP | pick the transport via `transport="hses"\|"ethernet"` |
| `HsesReplayServer` | — (testing) | replays a recorded robot session as a UDP server → offline development/testing |

Command coverage: HSES Robot Control **45/45**, File Control **5/5**
(verified against real hardware: YRC1000, SW YAS5.40).

## Installation

```bash
pip install yrcom           # from PyPI
```

For development:

```bash
pip install -e ".[dev]"     # editable install incl. pytest
```

## Quick start

```python
from yrcom import HsesClient

c = HsesClient("192.0.2.10")    # UDP 10040
c.open()
print(c.read_status())          # {'servo_on': False, 'teach': True, ...}
print(c.read_d_var(0))          # D[0]
c.close()
```

Transport-agnostic via `RobotConnection`:

```python
from yrcom import RobotConnection

rc = RobotConnection("192.0.2.10", transport="hses")
rc.connect()
print(rc.status())              # same keys, whether HSES or ES
```

## Offline: the replay simulator

Record a controller's real responses as JSON (tx/rx pairs per command, see
`tests/fixtures/sample_capture.json` for the format) and replay them — the
real `HsesClient` talks to the simulator:

```python
from yrcom import HsesReplayServer, HsesClient

with HsesReplayServer.from_capture("capture.json") as srv:
    c = HsesClient("127.0.0.1", srv.port); c.open()
    print(c.read_status())      # recorded response, no robot needed
```

Or as a standalone server: `python -m yrcom.hses_sim capture.json --port 10040`.

## Legal

Independent community project, not affiliated with or endorsed by Yaskawa.
*Yaskawa*, *Motoman* and *YRC1000* are trademarks of Yaskawa Electric
Corporation and are used here descriptively (compatibility) only. The
implementation is based on the publicly available protocol manuals
(HW0483202, HW1483358).

## Tests

```bash
pip install -e ".[dev]"
pytest
```

The tests run **without a robot**: a stateful ES mock (`tests/mock_robot.py`)
and a small HSES capture fixture (`tests/fixtures/`).

## License

MIT — see [LICENSE](https://github.com/CMOSBK/yrcom/blob/main/LICENSE).
