# Build your own Cerberus

A field guide to building, flashing, and bringing up a CerberusCAN — written from an
actual first-light bring-up on a 2013 Audi A6 C7, so the gotchas that cost *us* time
don't cost *you* any.

> ⚠️ Alpha hardware tool. Head 1 (diagnostics) is validated on a real car. The **write /
> CP / EMU** paths are bench/experimental — don't point write features at a vehicle you
> can't recover.

## 1. Bill of materials

| Part | Qty | Notes |
|------|-----|-------|
| Teensy 4.1 (NXP i.MX RT1062) | 1 | the brain (~$30, PJRC) |
| `SN65HVD230` CAN transceiver board (3.3 V) | **2** | Head 1 (active VCI) **and** Head 2 (listen-only logger) — both tap the *same* diagnostic bus |
| OBD-II (J1962) male connector or pigtail | 1 | a pre-made pigtail saves crimping fiddly terminals |
| Hookup wire | — | 26–28 AWG stranded if you're stuffing an OBD shell — it's all milliamps |
| USB cable (**data**, not charge-only) | 1 | powers *and* talks to the Teensy |
| 0.91"/0.96" SSD1306 OLED (I²C) | opt | status HUD — auto-detected, no config |
| Enclosure / OBD shell | opt | |

You'll also want a **soldering iron** and — trust us — a **multimeter** for bring-up.

## 2. Wire it

Full pinout and power notes: **[HARDWARE.md](HARDWARE.md)**. Both transceivers tap the
**same diagnostic bus (OBD 6/14)** — Head 1 drives it, Head 2 listens:

| Wire | Head 1 (active VCI) | Head 2 (logger) |
|---|---|---|
| board CANH | OBD **6** | OBD **6** |
| board CANL | OBD **14** | OBD **14** |
| board VCC | Teensy **3V3** | Teensy **3V3** |
| board GND | Teensy GND **+ OBD 4** | Teensy GND **+ OBD 4** |
| board CTX | Teensy **22** | Teensy **1** |
| board CRX | Teensy **23** | Teensy **0** |

Optional OLED: SDA→**18**, SCL→**19**, VCC→**3V3**, GND→**GND**.

- **Power is USB** — the Teensy's 3.3 V feeds both transceivers. No 12 V needed for bench/read use.
- **Remove the 120 Ω terminator on *both* boards** — the car's bus is already terminated (~60 Ω);
  two extra terminators sag it (3 × 120 Ω ≈ 40 Ω → errors). The two taps must add none.
- **Common ground is mandatory** — OBD pin 4 must tie to the Teensy/transceiver ground, or both heads go deaf.
- **Head 1 transmits; Head 2 is held listen-only** in firmware — it never ACKs or drives, it just
  logs the wire while Head 1 works (the `MON` command).

### ⚠️ THE gotcha — clone boards mislabel CTX/CRX

This cost us an hour. Many cheap `SN65HVD230` boards have **mislabeled or UART-style TX/RX silk.**
If at bring-up `INFO`/`PING` work but **no module answers**, your data pair is crossed at the chip —
**swap the two wires on Teensy 22 ↔ 23** (and 1 ↔ 0 for Head 2). On our board "straight-through per
the silk" was actually crossed; swapping fixed it instantly. (CAN is a differential bus, *not* a
UART — there's no real TX↔RX crossover; the silk is just wrong.)

## 3. Flash

### Option A — pre-built firmware (no toolchain)

1. Download **[`firmware/cerberus-can-teensy41.hex`](../firmware/cerberus-can-teensy41.hex)**
   (always the latest build; `INFO` reports the exact version it shipped with).
2. Install the **[Teensy Loader](https://www.pjrc.com/teensy/loader.html)** app.
3. Open the `.hex`, plug in the Teensy, press the **program button**. Done.

### Option B — build from source (PlatformIO)

```bash
pip install platformio            # if you don't already have it
git clone https://github.com/dspl1236/CerberusCAN
cd CerberusCAN
python -m platformio run -t upload
```
First build pulls the Teensy toolchain (~a few hundred MB, one-time). **If the upload prints
*"press the program button"*** it couldn't auto-reboot (usually a serial monitor still holding the
COM port) — close any monitor, press the Teensy's program button, or unplug/replug and re-run.

## 4. Bring-up (do this *before* the car)

Plug the Teensy into USB — with the dual-serial firmware it enumerates as **two COM ports**
(product string **`CerberusCAN`** / `Orthrus`): a smart command/CAN port and an always-dumb K-line
cable. Open the **smart** port at **115200** and type:

```
INFO   ->  CERBERUS:0.9.16 board=T4.1 product=Cerberus CAN1=500000 CAN2=500000 tmo=6000 respmax=4096 monring=16384 kline2=raw
PING   ->  PONG
```
*(On the raw K-line port, `INFO` gets no reply — it's a transparent cable. The Console's K-Line tab
labels which COM is which.)*

That proves the firmware's alive and both heads are on the 500 k diag bus. **It says nothing about
the CAN wiring** — `INFO`/`PING` only exercise the PC↔Teensy USB link, upstream of the transceivers.

## 5. On the car

1. Plug into the OBD-II port. **Ignition on.**
2. Ping a module: `710:77A:3E00` (TesterPresent → gateway). A reachable module replies `OK:7E00`.
3. Start the logger: **`MON:on`** — Head 2 now records the bus; `MON:stat` shows frame/peak/dropped.
4. Or run a host script: `pip install pyserial && python host/cerberus_probe.py COMx`.

### Expect `SNIFF`/`MON` to show ~nothing at idle — that's normal

The OBD diagnostic CAN (6/14) on VAG cars is **silent until you talk to it** — it carries tester ↔
gateway traffic, not broadcasts. So 0 frames at idle is expected; the logger lights up when *you*
issue commands on Head 1 or a tester (ODIS / VCDS) runs. A module *answering* is the real proof.

## 6. Troubleshooting — everything we actually hit

| Symptom | Cause / fix |
|---|---|
| `pio` not recognized | `pip install platformio`, then `python -m platformio run -t upload` |
| No COM port appears | Use a **data** USB cable (not charge-only). |
| Upload prints "press the program button" / no board found | Auto-reboot didn't catch (a serial monitor held the port). Close monitors, press the program button, or unplug/replug and re-run. The board isn't harmed. |
| `INFO`/`PING` work, but every module times out | Physical bus path. In order: **3.3 V** at each board VCC → **OBD 4 grounded** to common → CANH/CANL continuity to OBD 6/14 → then **swap CTX/CRX** (the #1 cause). |
| Modules time out right after bench testing | Bench TX with no bus pushes the controllers to bus-off — **replug the Teensy** for a clean boot (it self-recovers on a live bus anyway). |
| `ERR:no-flow-control` on a write | The target ECU never sent CTS — wrong ID, or wrong session. |

## 7. Validated

First-light on a **2013 Audi A6 C7 (3.0T)**: Head 1 reads live module data through the gateway —
VIN, part numbers, and a full module map (30 raw `SCAN` hits → 14 real modules) all pulled via
multi-frame ISO-TP. The dual-head logger firmware is flashed + verified on the board. The
write / CP / EMU paths remain bench-experimental.

MIT. Built for owners.
