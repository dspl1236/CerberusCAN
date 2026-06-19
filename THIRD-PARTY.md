# Third-party components

CerberusCAN's own code is **MIT** (see LICENSE). It builds on / ships alongside the
following, each under its own terms:

| Component | Where | License |
|---|---|---|
| **teensy_loader_cli** | bundled in `CerberusConsole.exe` as a separate invoked binary (mere aggregation) | **GPLv3** — see `host/tools/LICENSE-teensy_loader_cli.txt` / https://github.com/PaulStoffregen/teensy_loader_cli |
| **Teensyduino core** | firmware (PlatformIO dependency, not vendored) | PJRC license (commercial use permitted) |
| **FlexCAN_T4** | firmware (PlatformIO dependency) | MIT |
| **SSD1306Ascii** | firmware (optional OLED, PlatformIO dependency) | MIT |
| **DIDB fault/measuring data** (`host/didb/`) | dealer-derived VAG fault-location data, ported from KWPBridge | data — attribution kept in `host/didb/CREDITS.md`; not covered by the MIT code license |
| **SAE J2012 P-code tables** (`host/saedb/`) | generated reference text | reference data |

Bundling the GPLv3 `teensy_loader_cli` as a separately-invoked binary is *aggregation*,
which does not place CerberusCAN's MIT code under the GPL. Its license travels with it.
