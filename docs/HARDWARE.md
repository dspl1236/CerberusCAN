# Cerberus — Hardware

## OBD-II (J1962) tap

Both heads tap the **same diagnostic bus** — Head 1 drives it (active VCI), Head 2 is a
*second* `SN65HVD230` held listen-only as the always-on logger.

| OBD pin | Signal | Head | Wire to |
|---|---|---|---|
| 6  | CAN-H diagnostic (500 k) | 1 **+ 2** | both transceivers' CANH |
| 14 | CAN-L diagnostic         | 1 **+ 2** | both transceivers' CANL |
| 4  | chassis ground | — | common GND (mandatory) |
| 16 | +12 V (always hot) | — | *optional* power-in for standalone use (regulate to 5 V) |

Connector numbering (looking at the car's socket): two rows of 8, pin 1 top-left, pin 8
top-right, pin 9 bottom-left, pin 16 bottom-right. So **6** is top row (4th from right) and
**14** is bottom row (3rd from right).

Power the Teensy from **USB** on the bench — only take 6/14 and a ground from the car. No
pin 16 needed unless you want it standalone.

## Teensy 4.1 FlexCAN pins (fixed by the i.MX RT1062)

| Head | Controller | TX | RX | Role |
|---|---|---|---|---|
| 1 | CAN1 | 22 | 23 | **active VCI** — classic CAN 2.0B, 500 k |
| 2 | CAN2 | 1  | 0  | **listen-only logger** — classic CAN 2.0B, 500 k (2nd tap on 6/14) |
| 3 | CAN3 | 31 | 30 | **spare / plug-in port** — CAN-FD capable |

## Power

Both transceiver boards are **3.3 V** `SN65HVD230`s — they run straight off the Teensy's
**3V3** pin (board VCC → 3V3, board GND → GND). No 5 V, no level shifting, no VIO. Power the
whole rig from the laptop's **USB**; the Teensy's regulator feeds both boards (~20 mA total).
Only reach for OBD pin 16 (+12 V → a 5 V buck → Teensy VIN) if you want it standalone with no
laptop — and then **cut the Teensy's VUSB/VIN pad** so USB-data and the buck don't both push
5 V onto VIN.

## Wiring direction (the #1 gotcha — this one bit us)

Board **CTX → Teensy CAN-TX** (22 / 1), board **CRX → Teensy CAN-RX** (23 / 0) — straight
through *by the silk*, because CAN is a differential bus, not a UART peer (no TX↔RX crossover
like serial or fiber). **BUT cheap `SN65HVD230` clones frequently mislabel CTX/CRX.** Our
"straight-through per the silk" was actually crossed at the chip, and the symptom was
nasty-but-specific: `INFO`/`PING` worked, power/ground checked out, the tap had continuity —
yet **every module timed out.** The fix was to **swap the two data wires on Teensy 22 ↔ 23**
(and 1 ↔ 0 for Head 2). Hit that exact symptom set? Swap the data pair first — 30 seconds.

## Termination

The vehicle bus is already terminated (~60 Ω). The Waveshare `SN65HVD230` boards each carry
their own **120 Ω** terminator — **disable/desolder it on *both* boards** (or their jumpers)
before tapping. Two extra 120 Ω in parallel with the car's 60 Ω drops the bus to ~30 Ω →
errors. When you're only tapping/sniffing, add **no** termination of your own.

## Transceivers

- **Head 1 + Head 2 (diagnostic bus, 500 k):** 3.3 V `SN65HVD230` — talks to the Teensy
  directly. (`TJA1051T/3` or `MCP2562` also work, but those are 5 V parts that need their
  3.3 V VIO pin tied to 3V3.) Head 2 is the *same part* as Head 1; the firmware just holds it
  listen-only.
- **Head 3 (spare, pins 30/31):** bring-your-own transceiver per the bus you want — see below.
  The `SN65HVD230` is a classic 1 Mbps part and is out of spec at CAN-FD data rates.

## Head 3 — the comfort-bus / CAN-FD plug-in (future)

Head 3 is a "bring-your-own-transceiver" port for buses the two diag-bus heads can't reach.
You usually **don't need it for CP work**: the comfort/convenience modules (HVAC, seats) are
reachable through the gateway on the 500 k diag bus, where **Head 2 logs them** (decoded over
VW TP 2.0). It's only for tapping a bus *directly*:

- **CAN-FD** (MQB-Evo / MLB-Evo, ~2018+) → an **FD-rated** transceiver (TI `TCAN1042` /
  `SN65HVD25x` family, 3.3 V). Classic-CAN cars (e.g. the C7) never use FD.
- **A direct LS-FT comfort bus** → a `TJA1055T/3` (fault-tolerant, 3.3 V VIO; **not** the
  5 V-logic `TJA1054A`/`AU5790`, whose 5 V RXD would over-volt the Teensy).

If you go for a direct comfort tap, **meter the target bus first** (it's an internal/OEM-discretion
bus, not the OBD port): car awake, back-probe the pair vs. ground —

- both ≈ **2.5 V** → high-speed CAN → use an `SN65HVD230`
- **split** (one ≈ 0 V, one ≈ 5 V) → low-speed fault-tolerant → use a `TJA1055T/3`
- **~0 V / floating** → no bus there

> Note: **OBD pins 3/11** on gateway cars (incl. the C7 A6) are **firewalled to the diagnostic
> CAN** — there's no comfort bus exposed on the OBD port
> ([Ross-Tech](https://forums.ross-tech.com/index.php?threads/18385/): the OBD CAN is "a
> dedicated diagnostic bus … separated from the car's internal CAN buses by gateway"). A direct
> comfort tap means an *internal* bus, not 3/11.

## Future: K-line / KWP2000 for pre-CAN cars

Older VAG cars (roughly pre-2004) speak **KWP2000 over K-line** (ISO 9141 / ISO 14230) on OBD
pin **7** — a single-wire 12 V bus, not CAN. Planned as a 4th interface; here's the design.

**The Teensy stays USB-powered — no DC-DC.** The only thing that touches the car's 12 V is the
K-line transceiver's *line side* (the bus idles high at battery voltage). That's one wire from
OBD pin 16 to the chip, not a converter.

```
OBD 7  (K-line) ───────────────┐
OBD 16 (+12 V) ─┬─[~510 Ω]──────┤   ← pull-up (recessive = 12 V)
                └───────────────┤   K-line transceiver  (line/Vs side off OBD 12 V)
 logic supply ──────────────────┤   (5 V or 3.3 V — see chip choice)
GND ────────────────────────────┘
                   TX / RX ──[opt. level shift]── Teensy spare UART (~10.4 kbaud)
```

**Chip choice — two options:**
- **`L9637D`** (ST) — the classic dedicated ISO-9141 K-line IC. **5 V logic**, so feed its Vcc from
  the Teensy's **VUSB** pin (USB 5 V) and put a bidirectional **level shifter** on TX/RX (the Teensy
  4.1 is *not* 5 V-tolerant). Most documented; clones exist as KKL breakouts.
- **`MC33660`** (NXP) — ISO K-line interface with a **3.3 V-capable logic supply**: run its logic off
  the Teensy **3.3 V** and wire TX/RX **straight to the UART — no level shifter, no USB-5 V tap.**
  The minimal-parts option (verify the logic-Vdd range on the datasheet first).
- *(A discrete N-FET + pull-up works but skips the protection/slew shaping — not recommended. LIN
  PHYs share the physical layer but are fiddly on 5-baud-init cars.)*

**Wires:** beyond the CAN heads' 6/14/4, just **OBD 7 (K-line)** + **OBD 16 (+12 V)**, plus TX/RX to
a free Teensy UART. K-line is *fewer* bus wires than CAN (one, not a pair).

**No battery-drain concern:** it's a plug-in diagnostic tool on USB power — unplug when done — and
K-line only works ignition-on anyway (the ECUs are asleep otherwise), so no sleep/auto-shutoff
logic is needed. Pin 16 (the only standard OBD power) being always-on is fine here.
