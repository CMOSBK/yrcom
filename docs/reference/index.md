# API reference

Every public class and function, generated directly from the docstrings in the
source. Pick a client:

- **[RobotConnection](connection.md)** — one IP, transport selectable by name.
- **[HsesClient](hses-client.md)** — HSES robot control (UDP 10040), the widest command set.
- **[YaskawaClient](ethernet-server.md)** — classic Ethernet Server host control (TCP 80).
- **[HsesFileClient](file-client.md)** — HSES File Control (UDP 10041): files, jobs, CMOS backup.
- **[Replay simulator](simulator.md)** — `HsesReplayServer` for offline development.
- **[Status codes](errors.md)** — decode HSES status and added-status bytes.

Everything below is re-exported from the top-level `yrcom` package, so
`from yrcom import HsesClient` is all you need.

::: yrcom
    options:
      members: false
      show_root_heading: false
      show_root_toc_entry: false
