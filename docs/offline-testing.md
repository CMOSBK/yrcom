# Offline testing

You don't need a robot on the bench to develop against `yrcom`. Record a
controller's real responses once, then replay them — the **real** `HsesClient`
talks to a local simulator that answers from the recording.

## Record, then replay

A capture is JSON: tx/rx byte pairs per command. See
[`tests/fixtures/sample_capture.json`](https://github.com/CMOSBK/yrcom/blob/main/tests/fixtures/sample_capture.json)
for the exact format.

```python
from yrcom import HsesReplayServer, HsesClient

with HsesReplayServer.from_capture("capture.json") as srv:
    c = HsesClient("127.0.0.1", srv.port)
    c.open()
    print(c.read_status())      # recorded response, no robot needed
    c.close()
```

Because the simulator speaks real HSES on a UDP socket, your code under test is
identical to production — only the IP and port point at `127.0.0.1`.

## As a standalone server

Run the simulator as a process and point any HSES client at it:

```bash
python -m yrcom.hses_sim capture.json --port 10040
```

There is also a console entry point installed with the package:

```bash
yrcom-sim capture.json --port 10040
```

## In the test suite

The project's own tests run **without a robot**: a stateful Ethernet Server mock
(`tests/mock_robot.py`) plus small HSES capture fixtures under `tests/fixtures/`.
See [Contributing](contributing.md) for how to run them.
