#!/usr/bin/env python3
"""Dump everything a connected Teensy exposes over USB, so we can see whether a
*blank/factory* board reveals its model (4.0 vs 4.1) in the descriptor.

Run it the moment a new Teensy is plugged in (factory blink firmware, never flashed):
    python detect_teensy.py

If the VID/PID/serial/product/manufacturer differ between a 4.0 and a 4.1, we can
auto-detect blank boards in the Console and retire the manual picker. If they're
identical, the picker stays. This script settles it empirically.
"""
import sys

try:
    from serial.tools import list_ports
except Exception:
    print("pyserial not installed:  pip install pyserial")
    sys.exit(1)

PJRC_VID = 0x16C0  # all Teensy/PJRC devices

def main():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Plug the Teensy in and re-run.")
        return
    print(f"{len(ports)} serial port(s):\n")
    for p in ports:
        teensy = (p.vid == PJRC_VID)
        tag = "  <-- PJRC/Teensy" if teensy else ""
        print(f"{p.device}{tag}")
        print(f"    description : {p.description}")
        print(f"    hwid        : {p.hwid}")
        print(f"    VID:PID     : "
              + (f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "(none)"))
        print(f"    serial_num  : {p.serial_number}")
        print(f"    manufacturer: {p.manufacturer}")
        print(f"    product     : {p.product}")
        print(f"    interface   : {p.interface}")
        print()
    print("Note the VID:PID / product / serial. Compare a 4.0 vs a 4.1 capture:")
    print("  - differ  -> we can auto-detect blank boards (drop the picker)")
    print("  - same    -> 4.0/4.1 are indistinguishable blank; keep the picker")

if __name__ == "__main__":
    main()
