# Build your own Cerberus

A field guide to building, flashing, and bringing up a CerberusCAN — written from an
actual first-light bring-up on a 2013 Audi A6 C7, so the gotchas that cost *us* time
don't cost *you* any.

> ⚠️ Alpha hardware tool. Head 1 (diagnostic) is validated on a real car; Head 2 and
> the UDS write path are experimental. Don't point the write features at a vehicle you
> can't recover.

## 1. Bill of materials

| Part | Qty | Notes |
|------|-----|-------|
| Teensy 4.1 (NXP i.MX RT1062) | 1 | the brain (~$30, PJRC) |
| SN65HVD230 CAN transceiver board (3.3 V) | 1–2 | 1 does the read (Head 1); 2 for Head 1 + Head 2 |
| OBD-II (J1962) male connector or pigtail | 1 | a pre-made pigtail saves crimping fiddly terminals |
| Hookup wire | — | 26–28 AWG stranded if you're stuffing an OBD shell — it's all milliamps |
| USB cable (**data**, not charge-only) | 1 | powers *and* talks to the Teensy |
| Enclosure / OBD shell | opt | |

You'll also want a **soldering iron** and — trust us — a **multimeter** for bring-up.

## 2. Wire it

Full pinout, connector numbering, and power notes: **[HARDWARE.md](HARDWARE.md)**. Summary:

| Wire | Head 1 | Head 2 |
|---|---|---|
| board CANH | OBD **6** | OBD **3** |
| board CANL | OBD **14** | OBD **11** |
| board VCC | Teensy **3V3** | Teensy **3V3** |
| board GND | Teensy GND **+ OBD 4** | Teensy GND **+ OBD 4** |
| board CTX | Teensy **22** | Teensy **1** |
| board CRX | Teensy **23** | Teensy **0** |

- **Power is USB** — the Teensy's 3.3 V feeds the transceivers. No 12 V needed for bench/read use.
- **Remove the 120 Ω terminator on each board** — the car's bus is already terminated; a third resistor over-terminates it.
- **Common ground is mandatory** — OBD pin 4 (and 5) must tie to the Teensy/transceiver ground, or both heads go deaf.

### ⚠️ THE gotcha — clone boards mislabel CTX/CRX

This cost us an hour. Many cheap SN65HVD230 boards have **mislabeled or UART-style TX/RX silk.** If at bring-up `INFO`/`PING` work but **no module answers and `SNIFF` shows 0 frames**, your data pair is crossed at the chip — **swap the two wires on Teensy 22 ↔ 23** (and 1 ↔ 0 for Head 2). On our board, "straight-through per the silk" was actually crossed; swapping fixed it instantly.

## 3. Flash

### Option A — pre-built firmware (no toolchain)

1. Download **[`firmware/cerberus-can-v0.2.0-teensy41.hex`](../firmware/cerberus-can-v0.2.0-teensy41.hex)**.
2. Install the **[Teensy Loader](https://www.pjrc.com/teensy/loader.html)** app.
3. Open the `.hex`, plug in the Teensy, press the **program button**. Done.

### Option B — build from source (PlatformIO)

```bash
pip install platformio            # if you don't already have it
git clone https://github.com/dspl1236/CerberusCAN
cd CerberusCAN
python -m platformio run -t upload
```
First build pulls the Teensy toolchain (~a few hundred MB, one-time). If upload stalls
at *"waiting for Teensy device…"*, press the program button once.

## 4. Bring-up (do this *before* the car)

Plug the Teensy into USB — it enumerates as **`USB Serial Device (COMx)`**. Note the COM
number (Device Manager → Ports). Open it at **115200** (Teensy Loader's Serial Monitor,
`pio device monitor`, PuTTY, …) and type:

```
INFO   ->  CERBERUS:0.2.0-wip CAN1=500000 CAN2=100000
PING   ->  PONG
```

Those prove the firmware's alive and both buses are configured. **They say nothing about
the CAN wiring** — `INFO`/`PING` only exercise the PC↔Teensy USB link, two hops upstream
of the transceivers.

## 5. On the car

1. Plug into the OBD-II port. **Ignition on.**
2. Ping a module: `710:77A:3E00` (TesterPresent → gateway). A reachable module replies `OK:7E00`.
3. Or run the read: `pip install pyserial && python host/cerberus_probe.py COMx`.

### Expect `SNIFF` to show **0 frames** on a VAG OBD bus — that's normal

The OBD diagnostic CAN (pins 6/14) on VAG cars is **silent until you talk to it** — it
carries tester ↔ gateway traffic, not broadcasts. So `SNIFF:1` reading 0 is expected; a
module *answering* TesterPresent is the real proof you're on the bus.

## 6. Troubleshooting — everything we actually hit

| Symptom | Cause / fix |
|---|---|
| `pio` not recognized | `pip install platformio`, then `python -m platformio run -t upload` |
| No COM port appears | Use a **data** USB cable (not charge-only). A Teensy running a non-Serial sketch shows as a HID device — flashing Cerberus fixes it. |
| `INFO`/`PING` work, but every module times out + `SNIFF` = 0 | Physical bus path. In order: **3.3 V** at each board VCC → **OBD 4 grounded** to common → CANH/CANL continuity to OBD 6/14 → then **swap CTX/CRX** (the #1 cause). |
| Modules time out right after bench testing | Bench TX with no bus pushes the CAN controllers to bus-off — **replug the Teensy** for a clean boot (it self-recovers on a live bus anyway). |
| `ERR:tx` on a write | No flow control received — the target ECU isn't answering (wrong ID, or wrong session). |

## 7. Validated

First-light on a **2013 Audi A6 C7 (3.0T)**: Head 1 reads live module data through the
gateway — VIN, part numbers, and the J533 CP constellation (`0x04A3`) all pulled cleanly
via multi-frame ISO-TP, with every module (gateway / engine / trans / HVAC / cluster)
reachable. Head 2 (comfort) and the CP write path remain experimental.

GPLv3. Built for owners.
