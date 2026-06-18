<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/cerberus-can-logo-dark.png">
    <img alt="CerberusCAN — Teensy 4.1 tri-CAN OBD interface for VAG" src="docs/brand/cerberus-can-logo-light.png" width="460">
  </picture>
</p>

<p align="center">
  <strong>Teensy 4.1 tri-CAN OBD VCI for VAG — drive an active diagnostic session and
  losslessly log the wire at the same time, from one plug.</strong>
</p>

> **Status — v0.8.2, pre-1.0.** Head 1 (active diagnostics) is hardware-confirmed on a
> **2013 Audi A6 C7**: reads VIN, part numbers, the gateway part, and maps **30 modules** over
> J533 via multi-frame ISO-TP. The dual-head logger firmware is flashed + verified on the board.
> The **write / CP / TC1796 bench** paths are implemented but bench/experimental — not validated
> end-to-end on a car. The **OLED** output is unverified (built to the API; no panel on the dev
> bench). Don't point write features at a car you can't recover.

Cerberus turns a Teensy 4.1 (NXP i.MX RT1062, 600 MHz Cortex-M7) into a **request-level** VAG
VCI: you hand it a UDS request and the *firmware* runs the whole ISO-TP transaction on-device.
Three independent FlexCAN controllers let it do what a normal single-channel adapter can't —
**run an active UDS/CP exchange on one head while losslessly logging the unmasked wire on a
second**, so you can capture a Component-Protection handshake *as you drive it*.

> Three heads guarding the gate — the gate being VAG's gateway (J533) and everything it
> hides behind Component Protection.

## The three heads

| Head | FlexCAN | Teensy pins | Role |
|------|---------|-------------|------|
| **1** | CAN1 | 22 / 23 | **Active VCI** — Diagnostic CAN (OBD **6/14**), 500 k. UDS read/write/CP, SCAN, RAW, CANX. |
| **2** | CAN2 | 0 / 1 | **Always-on logger** — a *2nd* `SN65HVD230` paralleled on the **same OBD 6/14**, held LISTEN-ONLY. Captures the bus while Head 1 drives (`MON`). |
| **3** | CAN3 | 30 / 31 | **Spare** — CAN-FD-capable / bring-your-own-transceiver (FD part, or a `TJA1055T/3` for an LS-FT comfort-bus tap). Stub. |

> **Why Head 2 is on 6/14, not a comfort bus.** The old "Head 2 = 100 k comfort CAN on OBD
> 3/11" design was dropped in v0.6.0. On gateway cars (incl. the C7) the OBD port is firewalled
> to the diagnostic CAN, so 3/11 is dead — *and* the gateway routes the comfort/CP traffic onto
> the 500 k diag bus anyway, where Head 2 now logs it (seat/HVAC decode over VW TP 2.0). A
> *direct* comfort-bus tap is a future Head-3 plug-in (`TJA1055T/3`). See [docs/HARDWARE.md](docs/HARDWARE.md).

## Features

- **Request-level UDS/ISO-TP on-device** — single + multi-frame, flow control, block-size/STmin,
  `0x78` response-pending, all handled on the Teensy. Writes need no special command (a payload
  >7 bytes is auto First-Frame/CF framed).
- **Dual-head capture** — active Head 1 + always-on listen-only Head 2 logger (`MON`), backed by
  a **256 KB OCRAM ring buffer** so a USB stall or a long blocking transaction never drops frames
  (`MON:stat` reports peak/dropped).
- **SLCAN (Lawicel) mode** — drops into the wider ecosystem: SavvyCAN, python-can, `slcand`→SocketCAN.
- **TC1796 CAN-BSL bench read/write** for **Simos 8.x** ECUs (`host/tc1796_bsl.py`) — the engine
  ECU that can't be flashed in-car. See [docs/tc1796-bsl-cerberus-port.md](docs/tc1796-bsl-cerberus-port.md).
- **Optional SSD1306 OLED HUD** (auto-detected) — live **mode** title + per-head **VU bars**.

## Flash

```bash
# A) no toolchain: open firmware/cerberus-can-0.8.2-teensy41.hex in Teensy Loader
# B) from source (PlatformIO):
pip install platformio
python -m platformio run -t upload
```

## Wiring

**Head 1 (active)** through a 3.3 V `SN65HVD230`: OBD 6→CANH, 14→CANL, 4→GND; board CTX→Teensy 22,
CRX→Teensy 23, VCC→3V3, GND→GND.

**Head 2 (logger)** — a *second* `SN65HVD230`: CANH→OBD 6, CANL→OBD 14 (paralleled on Head 1),
CTX→Teensy 1, CRX→Teensy 0, VCC→3V3, GND→GND.

- **Remove the 120 Ω terminator from *both* breakouts** — the car already has ~60 Ω; the taps must
  add none.
- **Tie Teensy GND to OBD/chassis GND.**
- *(Clone boards mislabel CTX/CRX — if no comms, swap the pair.)*

**OLED (optional):** SDA→18, SCL→19, VCC→3V3, GND→GND. Auto-detected at 0x3C/0x3D; 128×64 default
(`#define OLED_128x64`; comment for a 0.91" 128×32).

Full BOM + bring-up: **[docs/BUILD.md](docs/BUILD.md)**.

## Host protocol (USB serial, 115200)

ASCII, one command per line:

```
<TX>:<RX>:<HEX>               UDS on Head 1 (shorthand)            710:77A:2200BE
UDS:<bus>:<TX>:<RX>:<HEX>     full ISO-TP UDS on bus 1|2           UDS:1:710:77A:2E00BE…
RAW:<bus>:<ID>:<HEX>          send ONE classic frame (no ISO-TP)
CANX:<bus>:<ID>:<HEX>[:ms]    send one frame then listen ms        (drives the TC1796 BSL)
SCAN:<bus>[:lo:hi[:win]]      active responder sweep (TesterPresent)
SNIFF:<bus>:<ms>[:lo:hi]      passive LISTEN-ONLY dump
MON:on[:lo:hi] | off | stat   always-on Head-2 ring-buffered logger  -> M2:<ms>:<id>:<hex>[:OVR]
TP:<bus>:<TX>:<ms> | TP:STOP  background TesterPresent keep-alive
STATS:<bus>                   CAN error counters / bus health
SLCAN                         enter Lawicel mode on Head 1 (reset to exit)
INFO  /  PING

-> OK:<resphex> | ERR:<reason> | RX:<ms>:<id>:<data> … DONE:<n>
```

## Host scripts (`host/`)

- **`cerberus_sniff.py`** — live `SNIFF`/`MON` capture, per-ID summary, CSV out.
- **`cerberus_decode.py`** — turn a capture into labeled UDS/KWP exchanges (ISO-TP **+ VW TP 2.0**),
  flagging the CP-relevant services (TrainICA, `0x00BE` IKA write, SecurityAccess).
- **`tc1796_bsl.py`** — TC1796 CAN-BSL Tier-1 driver (Simos 8.x bench read/write); `--selftest` runs
  offline.
- **`cerberus_probe.py`** — Experiment 1: reads `0x00BE` across J533/J255/J136/J285 (VIN-bound vs
  module-bound verdict).
- **`cerberus_write.py`** — careful generic UDS write: read-before → confirm → `2E` → read-back, `--dry-run`.
- **`sample_capture.csv`** — a VIN-free real-C7 slice to try the decoder with **no hardware**.

**Seen in the wild:** a real [`SCAN` of a 2013 Audi A6 C7](docs/example-scan-c7-a6.md) — one head
mapped and named the whole car off the gateway.

## Roadmap

- [x] Head 1 @ 500 k — request-level ISO-TP/UDS read + write
- [x] Hardware listen-only sniff (LOM) — *v0.4.0*
- [x] Dual-head: active VCI + always-on logger (`MON`), ring-buffered lossless — *v0.6 / 0.7*
- [x] SLCAN ecosystem interop — *v0.4.0*
- [x] TC1796 CAN-BSL Tier-1 — Simos 8.x bench read/write — *v0.5.0*
- [x] OLED status HUD (mode + VU bars) — *v0.8.x*
- [x] Simos-Suite drives Cerberus (dual-head driver + CP Capture live view)
- [ ] Head 3 configurable tap channel (CAN-FD / `TJA1055T/3` comfort bus)
- [ ] `EMU` responder mode — fake a module to probe the gateway / CP from the other side
- [ ] Tier-2 BSL — on-board PWM + RST for the SBOOT boot-password extraction
- [ ] On-device SD logging

## Related

- [Simos-Suite](https://github.com/dspl1236/simos-suite) — VAG ECU/TCU tooling + CP work (drives Cerberus directly)
- [esp32-isotp-ble-bridge-c7vag](https://github.com/dspl1236/esp32-isotp-ble-bridge-c7vag) — the BLE/USB sibling bridge
- [VAG-CP-Docs](https://github.com/dspl1236/VAG-CP-Docs) — Component Protection research

GPLv3. Built for owners.
