# yrcom

Yaskawa robot communication as a standalone Python library — a single,
hardware-verified communication core meant to be shared across projects
instead of being duplicated.

The name: **YRC** (Yaskawa Robot Controller, e.g. YRC1000) + **COM**munication.

**Pure stdlib** — no runtime dependencies. Import name: `yrcom`.

## Transports

| Class | Transport | Purpose |
|---|---|---|
| [`YaskawaClient`](reference/ethernet-server.md) | Ethernet Server, TCP 80 | classic host-control commands |
| [`HsesClient`](reference/hses-client.md) | HSES, UDP 10040 | high speed: 45 commands (status, alarms, variables, IO, telemetry, motion) |
| [`HsesFileClient`](reference/file-client.md) | HSES File Control, UDP 10041 | read/write job files, file list, CMOS backup |
| [`RobotConnection`](reference/connection.md) | ES + HSES behind one IP | pick the transport via `transport="hses"\|"ethernet"` |
| [`HsesReplayServer`](reference/simulator.md) | — (testing) | replays a recorded robot session as a UDP server → offline development/testing |

Command coverage: HSES Robot Control **45/45**, File Control **5/5**
(verified against real hardware: YRC1000, SW YAS5.40).

## Where to go next

- **[Getting started](getting-started.md)** — install and make the first call.
- **[Transports](transports.md)** — which client to reach for, and why.
- **[Offline testing](offline-testing.md)** — develop without a robot on the bench.
- **[API reference](reference/index.md)** — every public class and method.

## Legal

Independent community project, not affiliated with or endorsed by Yaskawa.
*Yaskawa*, *Motoman* and *YRC1000* are trademarks of Yaskawa Electric
Corporation and are used here descriptively (compatibility) only. The
implementation is based on the publicly available protocol manuals
(HW0483202, HW1483358).

MIT licensed — see [LICENSE](https://github.com/CMOSBK/yrcom/blob/main/LICENSE).
