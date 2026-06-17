# TC1796 CAN-BSL on Cerberus — bench read/write of Simos 8.x

Goal: use one Cerberus (Teensy 4.1) instead of a Raspberry Pi + CAN hat to bench-read
and write **Continental Simos 8 / 8.4 / 8.5** ECUs (Infineon **TriCore TC1796 / AudoNG**),
e.g. the Audi C7 3.0T `4G0907551x`. The engine ECU can't be programmed over in-car OBD
(`10 02` is gated); the supported path is the bootloader, on the bench.

**Credit:** protocol reverse-engineering is bri3d (`TC1791_CAN_BSL`, Simos18/AudoMAX) and
fastboatster (`TC1796_CAN_BSL`, Simos8/AudoNG port), both GPL-3.0. `bootloader.bin`,
`twister`, and `crchack` come from those projects — get them there; we don't vendor them.

---

## Two tiers

| Tier | What you can do | Needs |
|------|-----------------|-------|
| **1 — read/write (passwords in hand)** | upload BSL, unlock flash, read words/regions, erase, write | firmware `CANX` (**done**) + 2–3 jumper wires. **Not timing-critical.** |
| **2 — extract passwords (first time)** | recover the per-ECU 16-byte read+write passwords | + firmware hardware-PWM + RST GPIO; host runs `twister`/`crchack`. **Timing-critical.** |

Tier 1 is implemented now (`host/tc1796_bsl.py`). Tier 2 is the next firmware step.

---

## Protocol reference (verified against fastboatster/TC1796_CAN_BSL)

CAN: **500 kbit/s**, classic 11-bit, on Head 1.

**Stage 1 — Mask-ROM CAN BSL upload** (only needs the HWCFG strap, not timing-critical):
```
init  → id 0x100  : 55 55 00 01 <nblocks LE16> 00 03      (nblocks = ceil(len(bootloader.bin)/8))
ACK   ← id 0x40                                            (ECU confirms BSL)
data  → id 0xC0   : bootloader.bin, 8 bytes/frame          (copied to SPRAM 0xD0004000, then run)
```

**Stage 2 — RAM-bootloader commands** (8-byte frames @ **id 0xC0**, 2-byte opcode `00 NN`,
2nd frame carries an XOR-of-15-header-bytes checksum trailer):
| op | meaning | frame1 | frame2 (last byte = XOR chksum) |
|----|---------|--------|----------------------------------|
| `00 08` | read 4 bytes | `00 08 <addr BE4> 00 00` | `00*7 <chk>` → resp = 4 bytes |
| `00 0A` | streamed read | `00 0A <addr BE4> <size hi2>` | `<size lo2> 00*5 <chk>` |
| `00 10` | send passwords | `00 10 <pw1 4> <pw2 hi2>` | `<pw2 lo2> 00 <rw> <ucb> 00 00 <chk>` (`rw` 0x00=read, 0x01=write); ok = resp[0]==0x55 |
| `00 04` | erase sector | `00 04 <addr BE4> <size hi2>` | `<size lo2> 00*5 <chk>` |
| `01 00..` | device id | one frame | two resp frames, 6 id bytes each |

Flash map (Simos 8.5): CBOOT `0xA0020000` (0x20000), **CAL `0xA0040000` (0x40000)**,
ASW `0xA0080000`/`0xA0100000`/`0xA0180000` (0x80000 each). Read window e.g. `A0000000..001FFFFF`.

**Stage 3 — password extraction (Tier 2, SBOOT exploit)** — timing-critical:
- PWM **3210 Hz** on two pins (50 % duty / 25 % @ ¾ phase) → harness 60-pin **55 / 40**; pulse RST.
- SBOOT shell = ISO-TP txid `0x7E0`/rxid `0x7E8`, pad `0x55`: `0x30` elevate → `0x54` gen-seed (256 B)
  → host `twister` (weak-MT + RSA-1024) → `0x65` + key → `0x78` CRC-config over OTP password range
  (`0x8001420C..18`) + a per-software **part-correlation string** → `0x79` run → read CRC back →
  host `crchack` infers 4 bytes/run × 4 = 16-byte password (8 read + 8 write).

---

## Wiring (Teensy 4.1 ↔ ECU, on the bench)

ECU **94-pin** harness: `68`=CAN-Hi, `67`=CAN-Lo, `64`+`87`=+12V, `2`=PSU GND, `1`=ctrl GND.
**Tie ECU ground to Teensy ground** (the "unbound grounds" gotcha, esp. for Tier-2 PWM).

| Cerberus | → | ECU |
|----------|---|-----|
| Head 1 CAN (SN65HVD230, pins 22/23) | → | 94-pin 68 / 67 |
| 3V3 rail | → | HWCFG strap PCB points (one to GND, one to 3V3 via 1k) + CAN-transmitter-wake point |
| 13.6 V bench PSU | → | 64/87 (+12V), 2 (GND); tie GND to Teensy GND |
| 2× PWM pins → level shifter | → | 60-pin 55 (PWM0) / 40 (PWM1)  *(Tier 2)* |
| 1× GPIO → level shifter | → | CPU RST PCB point  *(Tier 2)* |

3.3 V→5 V level shifters on PWM/RST (TriCore I/O). CAN handled by the HVD230.

---

## Firmware

**Done (v0.5.0):** `CANX:bus:ID:HEX[:waitms]` — send one classic frame in TX mode, then
listen `waitms` ms (`RX:<id>:<hex>` … `DONE:<n>`), or fire-and-forget (`waitms` omitted/0).
This is the send-then-listen primitive `SNIFF` (listen-only) and `RAW` (send-only) lacked;
the host driver builds the entire BSL protocol on it.

**TODO — Tier 2 / performance:**
- `PWM:freq:dutyA:phaseA:dutyB:phaseB` — FlexPWM on two pins (jitter-free vs the Pi's pigpio
  software waves — the reference rig's main reliability pain). 3210 Hz default.
- `RST:pin:ms` — pulse a GPIO for the ECU reset.
- On-device streamed read (`BSLREAD addr len`) so multi-MB dumps don't go word-by-word over serial.

---

## Host tool — `host/tc1796_bsl.py`

```
python tc1796_bsl.py --selftest                          # offline framing check (no hardware)
python tc1796_bsl.py COM12 upload --bin bootloader.bin   # Stage 1
python tc1796_bsl.py COM12 readid
python tc1796_bsl.py COM12 unlock-read ABF425508513C273  # 8-byte read password
python tc1796_bsl.py COM12 read A0040000                 # one 32-bit word
python tc1796_bsl.py COM12 readmem A0040000 1000 cal.bin # small region (slow, reliable)
```
`upload` → `unlock-read` → `read`/`readmem` is the Tier-1 proof chain.

---

## Caveats / honesty
- **Not hardware-tested yet** — framing is unit-verified (`--selftest` passes), but the on-ECU
  handshake is unproven until run on the bench. Treat as a first cut; expect iteration.
- `readmem` is word-by-word (one serial round-trip per 4 bytes) — fine for the password area /
  identity / small CAL chunks; a full 2 MB read wants the on-device streamed read (TODO above).
- Upload streams ~900 frames paced by serial round-trips; if the BSL times out between blocks,
  the fix is an on-device `BSLUP` (v2). 
- The Tier-2 **part-correlation string** is per-software — derive ours from the `4G0907551D`
  ASW part before attempting extraction.
- Byte order of `read_addr` output assumed memory-order; verify against a known word on first run.
- This is **ECU-out, case-open, bench** work. Back up before erasing/writing.
