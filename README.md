# Cerberus 🐕‍🦺

**Teensy 4.1 tri-CAN OBD interface for VAG — diagnostic + comfort bus from one plug.**

> 🚧 **Work in progress.** Firmware implements Head 1 (read **and** write UDS) and Head 2 (sniff), but is **not yet hardware-tested**. Head 3 (CAN-FD) is a stub. Expect rough edges — don't trust it against a car you can't recover.

Cerberus turns a Teensy 4.1 (NXP i.MX RT1062) into a multi-bus VW/Audi OBD tool.
Three CAN heads, one OBD connector — read diagnostics on the powertrain bus *and*
sniff the convenience bus (where Component Protection lives) at the same time.

> Three heads guarding the gate — the gate being VAG's gateway (J533) and
> everything it hides behind Component Protection.

## The three heads

| Head | Bus | OBD pins | Rate | Transceiver | Status |
|------|-----|----------|------|-------------|--------|
| **1** | Powertrain / Diagnostic | **6 / 14** | 500 kbps | high-speed — `SN65HVD230` | ✅ read + write |
| **2** | Comfort / Convenience | **3 / 11** | 100 kbps | HS *or* FT — measure (see below) | ⚙️ sniff + UDS |
| **3** | spare (CAN-FD) | **30 / 31** | — | FD-rated (**not** `SN65HVD230`) | 🔧 stub |

Head 1 alone does the whole UDS job: the gateway (J533) routes diagnostic
requests to every module, so one 500 kbps bus reaches J533 / J255 / J136 / J285.
Head 2 taps the convenience bus directly to watch the J533 ↔ J255 CP handshake.

> ⚠️ **Head 2's transceiver depends on the bus.** VAG's 100 kbps comfort bus is
> *sometimes* **low-speed fault-tolerant CAN** (ISO 11898-3) — a different physical
> layer. Measure idle volts on OBD 3/11: **both ≈ 2.5 V** → high-speed, a second
> `SN65HVD230` works; **split (≈0 V / ≈5 V)** → fault-tolerant, use a `TJA1055T/3`
> (FT, 3.3 V VIO) instead. It's a *voltage-levels* thing, not a bit-rate thing.
> See [docs/HARDWARE.md](docs/HARDWARE.md).

## Quick start

```bash
# build + flash (PlatformIO)
pio run -t upload

# wire Head 1 through a 3.3 V SN65HVD230: OBD 6 -> CANH, 14 -> CANL, 4 -> GND;
# board CTX -> Teensy 22, CRX -> Teensy 23, VCC -> 3V3, GND -> GND. USB powers it.
# (disable the board's on-board 120 Ohm -- the car's bus is already terminated.)

# run the cross-module IKA read over USB
pip install pyserial
python host/cerberus_probe.py COM5
```

## Host protocol (USB serial, 115200)

ASCII, one command per line — Cerberus does all the ISO-TP framing:

```
<TXID>:<RXID>:<HEX>             UDS on Head 1        710:77A:2200BE
UDS:<bus>:<TXID>:<RXID>:<HEX>   UDS on bus 1|2       UDS:1:710:77A:2E00BE…
SNIFF:<bus>:<ms>                passive dump (ms=0 = until a serial byte)
INFO                            firmware + bus config
PING                            -> PONG

-> OK:<resphex> | ERR:<reason> | RX:<ms>:<id>:<data> … DONE:<n>
```

Single/multi-frame ISO-TP, flow control, block-size / STmin, and `0x78`
response-pending are handled on the Teensy. **Writes need no special command** — a
payload over 7 bytes (e.g. `2E00BE` + 34 = 37) is auto First-Frame/Consecutive-Frame
framed. The PC just sends UDS payloads: `1003`, then `2200BE` to read or `2E00BE…` to write.

## Roadmap

- [x] Head 1 @ 500k — ISO-TP / UDS bridge, **read + write** (runs Experiment 1, the `0x00BE` read)
- [x] Multi-frame ISO-TP TX — IKA / constellation writes (`2E00BE`, `2E04A3`)
- [x] Head 2 @ 100k — convenience-bus **sniffer** (`SNIFF:2:<ms>`) for CP handshake capture
- [ ] Hardware listen-only (truly silent sniff at an unknown baud)
- [ ] Head 3 — CAN-FD (MQB-Evo / MLB-Evo), FD-rated transceiver on pins 30/31
- [ ] Converge the host protocol with `esp32-isotp-ble-bridge-c7vag` so
      Simos-Suite's transport layer drives Cerberus unchanged

## Related

- [Simos-Suite](https://github.com/dspl1236/simos-suite) — VAG ECU/TCU tooling + CP work
- [esp32-isotp-ble-bridge-c7vag](https://github.com/dspl1236/esp32-isotp-ble-bridge-c7vag) — the BLE/WiFi sibling bridge
- [VAG-CP-Docs](https://github.com/dspl1236/VAG-CP-Docs) — Component Protection research

GPLv3. Built for owners.
