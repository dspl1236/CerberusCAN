# Product line — Orthrus & Cerberus

Two SKUs, **one codebase**. Same firmware source (two PlatformIO build targets) and the same
Console — the board decides which heads light up. Named for the hounds of Typhon & Echidna:
**Orthrus** the two-headed dog, **Cerberus** the three-headed guardian.

| | **Orthrus** | **Cerberus** |
|---|---|---|
| Board | Teensy **4.0** (compact, cheaper) | Teensy **4.1** (flagship) |
| Heads | 2 — **KWP · CAN** | 3 — **KWP · CAN · DoIP** |
| K-line / KWP (UART) | ✓ | ✓ |
| CAN — classic + FD | ✓ | ✓ |
| **DoIP** (Ethernet) | ✗ (no Ethernet pins on the 4.0) | ✓ |
| VAG eras | ~1995 → ~2018 (K-line → classic/FD CAN) | every era, '90s → today |
| `INFO` reports | `board=T4.0 product=Orthrus` | `board=T4.1 product=Cerberus` |
| Bundled hex | `firmware/cerberus-can-teensy40.hex` | `firmware/cerberus-can-teensy41.hex` |

## Why one repo

- **No fork.** `platformio.ini` already builds both targets from the same source
  (`pio run -e teensy40` / `-e teensy41`). Orthrus is literally the 4.0 build.
- **One Console.** It reads `product=`/`board=` from `INFO`, titles itself accordingly
  (e.g. *"Orthrus Console v0.9.10 — T4.0 fw 0.9.10"*), and will show the **DoIP page only on a 4.1**.
  Both SKUs stay in lockstep automatically — fix once, ship both.
- **The DoIP delta is a compile flag, not a branch.** The 4.0 has no Ethernet, so the DoIP head is
  wrapped in `#if defined(ARDUINO_TEENSY41)` — one source, board-conditional.
- **Shared data + docs** (DIDB, SAE tables, build pipeline) live once.

## OBD-II tap map (single J1962 connector)

| Pins | Bus | Head |
|---|---|---|
| 6 / 14 | CAN-H / CAN-L (classic **and** CAN-FD) | CAN (HVD230 classic; FD transceiver for FD) |
| 7 | K-line | KWP (Serial2 + K-line transceiver) |
| 3 / 11 + 12 / 13 | DoIP Ethernet, **100BASE-TX** (ISO 13400-4) | DoIP (DP83825 PHY) — **Cerberus/4.1 only** |
| 8 | DoIP activation line | GPIO — Cerberus only |
| 4 / 5 / 16 | grounds / +12 V | — |

DoIP at the OBD connector is standard **100BASE-TX**, so the 4.1's Ethernet PHY wires straight to
3/11/12/13 — no media converter. (The in-vehicle 100BASE-T1 backbone never reaches the diag port.)

## Status

Built + on the flagship today: KWP (firmware, bench-untested) + dual-CAN (hardware-validated).
**DoIP is the only unbuilt head** — parked until a 4.1 + Ethernet-PHY kit is wired to OBD 3/11/12/13
and a DoIP-era car is on hand. See [HARDWARE.md](HARDWARE.md) and the project README.
