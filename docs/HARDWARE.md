# Cerberus — Hardware

## OBD-II (J1962) tap

| OBD pin | Signal | Head | Wire to |
|---|---|---|---|
| 6  | CAN-H powertrain/diag (500k) | 1 | **HS** transceiver CANH |
| 14 | CAN-L powertrain/diag        | 1 | **HS** transceiver CANL |
| 3  | CAN-H comfort/convenience (100k) | 2 | **FT** transceiver CANH |
| 11 | CAN-L comfort/convenience        | 2 | **FT** transceiver CANL |
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

## Transceivers — do NOT mix these up

- **Head 1 (diagnostic, 500k):** high-speed CAN, ISO 11898-2 —
  `SN65HVD230` (3.3 V), `TJA1051`, `MCP2562`.
- **Head 2 (comfort, 100k):** **low-speed fault-tolerant** CAN, ISO 11898-3 —
  `TJA1054A`, `AU5790`, `PCA82C252`. A high-speed transceiver will **not** talk
  on this bus.

### Verify Head 2's physical layer before buying its transceiver

Back-probe OBD pins 3/11 vs. ground with the car awake:
- both ≈ **2.5 V** at idle  → high-speed (use a normal transceiver)
- **split** (one ≈ 0 V, one ≈ 5 V) → low-speed fault-tolerant (use the FT part)

The vehicle buses are already terminated (120 Ω HS / split-termination FT) — don't
add your own termination when only tapping/sniffing.
