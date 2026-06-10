# Cerberus 🐕‍🦺

**Teensy 4.1 tri-CAN OBD interface for VAG — diagnostic + comfort bus from one plug.**

> 🚧 **Work in progress.** The Head 1 firmware is written but **not yet hardware-tested**, and Heads 2 & 3 are stubs. This is an early scaffold — expect rough edges, and don't trust it against a car you can't recover.

Cerberus turns a Teensy 4.1 (NXP i.MX RT1062) into a multi-bus VW/Audi OBD tool.
Three CAN heads, one OBD connector — read diagnostics on the powertrain bus *and*
sniff the convenience bus (where Component Protection lives) at the same time.

> Three heads guarding the gate — the gate being VAG's gateway (J533) and
> everything it hides behind Component Protection.

## The three heads

| Head | Bus | OBD pins | Rate | Transceiver | Status |
|------|-----|----------|------|-------------|--------|
| **1** | Powertrain / Diagnostic | **6 / 14** | 500 kbps | high-speed (ISO 11898-2) | ✅ MVP |
| **2** | Comfort / Convenience | **3 / 11** | 100 kbps | **fault-tolerant** (ISO 11898-3) | 🔧 stub |
| **3** | spare (CAN-FD) | — | — | — | 🔧 stub |

Head 1 alone does the whole UDS job: the gateway (J533) routes diagnostic
requests to every module, so one 500 kbps bus reaches J533 / J255 / J136 / J285.
Head 2 taps the convenience bus directly to watch the J533 ↔ J255 CP handshake.

> ⚠️ **Transceiver warning:** Head 2's 100 kbps VAG comfort bus is **low-speed
> fault-tolerant CAN** — a *different physical layer*. It needs an FT transceiver
> (`TJA1054A` / `AU5790`), **not** a high-speed one. A high-speed transceiver on
> pins 3/11 will sit deaf. See [docs/HARDWARE.md](docs/HARDWARE.md).

## Quick start

```bash
# build + flash (PlatformIO)
pio run -t upload

# wire Head 1: OBD pin 6 -> CANH, pin 14 -> CANL, pin 4 -> GND, through a
# high-speed CAN transceiver on Teensy pins 22(TX)/23(RX). Power Teensy from USB.

# run the cross-module IKA read over USB
pip install pyserial
python host/cerberus_probe.py COM5
```

## Host protocol (USB serial, 115200)

ASCII, one request per line — Cerberus does the ISO-TP framing:

```
TXID:RXID:REQHEX          e.g.  710:77A:2200BE
-> OK:6200BE<34 bytes>    or    ERR:timeout
```

Single- and multi-frame ISO-TP, flow control, and `0x78` response-pending are
handled on the Teensy. The PC just sends UDS payloads (e.g. `1003` then `2200BE`).

## Roadmap

- [x] Head 1 @ 500k — ISO-TP / UDS bridge (runs Experiment 1, the `0x00BE` read)
- [ ] Head 2 @ 100k FT — convenience-bus sniffer (CP challenge/response capture)
- [ ] Head 3 — CAN-FD (MQB-Evo / MLB-Evo)
- [ ] Converge the host protocol with `esp32-isotp-ble-bridge-c7vag` so
      Simos-Suite's transport layer drives Cerberus unchanged

## Related

- [Simos-Suite](https://github.com/dspl1236/simos-suite) — VAG ECU/TCU tooling + CP work
- [esp32-isotp-ble-bridge-c7vag](https://github.com/dspl1236/esp32-isotp-ble-bridge-c7vag) — the BLE/WiFi sibling bridge
- [VAG-CP-Docs](https://github.com/dspl1236/VAG-CP-Docs) — Component Protection research

GPLv3. Built for owners.
