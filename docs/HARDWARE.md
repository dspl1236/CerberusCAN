# Cerberus — Hardware

## OBD-II (J1962) tap

| OBD pin | Signal | Head | Wire to |
|---|---|---|---|
| 6  | CAN-H powertrain/diag (500k) | 1 | **HS** transceiver CANH |
| 14 | CAN-L powertrain/diag        | 1 | **HS** transceiver CANL |
| 3  | CAN-H comfort/convenience (100k) | 2 | **HS** transceiver CANH (SN65HVD230) |
| 11 | CAN-L comfort/convenience        | 2 | **HS** transceiver CANL (SN65HVD230) |
| 4  | chassis ground | — | common GND |
| 16 | +12 V (always hot) | — | *optional* power-in (regulate to 5 V) |

Connector numbering (looking at the car's socket): two rows of 8, pin 1 top-left,
pin 8 top-right, pin 9 bottom-left, pin 16 bottom-right. So **6** is top row (4th
from right) and **14** is bottom row (3rd from right); **3** top row, **11** bottom row.

Power the Teensy from **USB** on the bench — only take 6/14 (+3/11) and a ground
from the car. No pin 16 needed unless you want it standalone.

## Teensy 4.1 FlexCAN pins (fixed by the i.MX RT1062)

| Head | Controller | TX | RX | Notes |
|---|---|---|---|---|
| 1 | CAN1 | 22 | 23 | classic CAN 2.0B |
| 2 | CAN2 | 1  | 0  | classic CAN 2.0B @ 100k |
| 3 | CAN3 | 31 | 30 | CAN-FD capable |

## Power

Both transceiver boards are **3.3 V** `SN65HVD230`s — they run straight off the
Teensy's **3V3** pin (board VCC → 3V3, board GND → GND). No 5 V, no level shifting,
no VIO. Power the whole rig from the laptop's **USB**; the Teensy's regulator feeds
both boards (~20 mA total). Only reach for OBD pin 16 (+12 V → a 5 V buck → Teensy
VIN) if you want it standalone with no laptop — and then **cut the Teensy's VUSB/VIN
pad** so USB-data and the buck don't both push 5 V onto VIN.

## Wiring direction (the #1 gotcha — this one bit us)

Board **CTX → Teensy CAN-TX** (22 / 1), board **CRX → Teensy CAN-RX** (23 / 0) —
straight through *by the silk*, because the transceiver is a level converter, not a
UART peer (no TX↔RX crossover like fiber or serial).

**BUT cheap SN65HVD230 clones frequently mislabel CTX/CRX.** On a real bring-up our
"straight-through per the silk" was actually crossed at the chip, and the symptom was
nasty-but-specific: `INFO`/`PING` worked, power/ground checked out, the bus tap had
continuity — and yet **every module timed out and `SNIFF` showed 0 frames.** The fix
was to **swap the two data wires on Teensy 22 ↔ 23** (and 1 ↔ 0 for Head 2). Hit that
exact symptom set? Swap the data pair first — it costs 30 seconds.

## Transceivers — do NOT mix these up

- **Head 1 (diagnostic, 500k):** high-speed CAN (ISO 11898-2). A 3.3 V `SN65HVD230`
  is the simplest — it talks to the Teensy directly. (`TJA1051T/3` or `MCP2562` also
  work, but those are 5 V parts that need their 3.3 V VIO pin tied to 3V3.)
- **Head 2 (comfort, 100k):** a second `SN65HVD230`, same as Head 1. This assumes the
  comfort bus runs **high-speed** CAN at 100k. If Head 2 captures nothing, that bus is
  **low-speed fault-tolerant** (ISO 11898-3) — a physical layer the HVD230 can't read —
  and *that head* would need an FT part (`TJA1055T/3`, FT with a 3.3 V VIO pin; **not**
  the 5 V-logic `TJA1054A` / `AU5790`, whose 5 V RXD would over-volt the Teensy).
- **Head 3 (CAN-FD, pins 30/31):** needs an **FD-rated** transceiver (TI `TCAN33x` /
  `SN65HVD25x` family for 3.3 V). The `SN65HVD230` is a 1 Mbps *classic* part and is
  out of spec at FD data rates — do **not** use it here. Only MQB-Evo / MLB-Evo cars
  use CAN-FD; classic-CAN cars (e.g. C7) never touch this head.

### Head 2 / OBD 3/11 is car-dependent — meter it first

Pins 3/11 are OEM-discretion. *Some* VAG/Porsche models route a second CAN there;
**many gateway cars (incl. the C7 A6) firewall the OBD port to the diagnostic CAN only**
— on those, 3/11 is dead ([Ross-Tech](https://forums.ross-tech.com/index.php?threads/18385/):
the OBD CAN is "a dedicated diagnostic bus … separated from the car's internal CAN buses
by gateway"). Back-probe OBD 3/11 vs. ground, car awake:
- both ≈ **2.5 V** → high-speed CAN present → Head 2 works (`SN65HVD230` is correct)
- **split** (one ≈ 0 V, one ≈ 5 V) → low-speed fault-tolerant → swap that head to a `TJA1055T/3`
- **~0 V / floating** → no bus on 3/11 (gateway-firewalled) → tap an *internal* bus for the comfort/CP handshake, not the OBD port

### Termination

The vehicle buses are already terminated (~60 Ω). The Waveshare `SN65HVD230` boards
carry their own **120 Ω** terminator — **disable/desolder it** (or its jumper) before
tapping, or you over-terminate the bus (3 × 120 Ω ≈ 40 Ω → errors). Add none of your
own when only tapping/sniffing.
