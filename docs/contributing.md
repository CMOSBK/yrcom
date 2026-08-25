# Contributing

## Development setup

```bash
git clone https://github.com/CMOSBK/yrcom
cd yrcom
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
```

## Running the tests

```bash
pytest
```

The suite runs **without a robot**: a stateful Ethernet Server mock
(`tests/mock_robot.py`) and small HSES capture fixtures under `tests/fixtures/`
stand in for the controller. This is the same replay mechanism described in
[Offline testing](offline-testing.md).

## Building the docs

```bash
pip install -e ".[docs]"
mkdocs serve          # live preview at http://127.0.0.1:8000
mkdocs build --strict # produce ./site, fail on warnings
```

The API reference is generated from docstrings via
[mkdocstrings](https://mkdocstrings.github.io/), so documenting a new public
method is just a matter of writing its docstring — no separate reference page to
maintain.

## Hardware verification

Command coverage is verified against real hardware (YRC1000, SW YAS5.40). When
you add or change a command, capture a real tx/rx exchange where possible and
extend the fixtures so the behaviour is covered offline too.

## Legal note

Independent community project, not affiliated with or endorsed by Yaskawa. The
implementation is based on the publicly available protocol manuals (HW0483202,
HW1483358). Keep contributions on that basis.
