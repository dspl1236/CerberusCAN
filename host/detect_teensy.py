#!/usr/bin/env python3
"""Show every Teensy/PJRC USB device on the machine — serial ports AND HID/bootloader/composite.

A serial-port scan (pyserial) only sees COM ports. But a Teensy out of the box runs PJRC's factory
program and enumerates as a *HID* device (no COM port), and in flash mode it's the HalfKay *HID*
bootloader — neither shows in a COM scan. So this also queries Windows' full USB device list (by the
PJRC vendor id 0x16C0) to catch them.

    python detect_teensy.py
"""
import subprocess
import sys

PJRC_VID = 0x16C0

# Known PJRC USB product ids (what the board is currently *doing*)
PJRC_PID = {
    0x0478: "HalfKay BOOTLOADER (flash mode)",
    0x0483: "USB Serial (single)",
    0x048B: "Dual Serial  <-- CerberusCAN/Orthrus firmware (smart + raw K-line)",
    0x0486: "HID composite (factory program / Keyboard-Mouse-Joystick type) - not serial",
    0x0489: "Triple Serial",
}


def serial_scan():
    print("=== Serial (COM) ports ===")
    try:
        from serial.tools import list_ports
    except Exception:
        print("  (pyserial not installed:  pip install pyserial)")
        return
    ports = list(list_ports.comports())
    if not ports:
        print("  none")
    for p in ports:
        teensy = "  <-- PJRC/Teensy" if p.vid == PJRC_VID else ""
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "(none)"
        print(f"  {p.device}: {vidpid}  {p.product or p.description}  sn={p.serial_number}{teensy}")


def usb_scan_windows():
    import re
    print("\n=== Connected PJRC (VID 16C0) USB devices ===")
    ps = (r"Get-PnpDevice | Where-Object { $_.InstanceId -match 'VID_16C0' -and $_.Status -eq 'OK' }"
          r" | Select-Object -ExpandProperty InstanceId")
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                             capture_output=True, text=True, timeout=30).stdout
    except Exception as e:
        print("  (couldn't run PowerShell:", e, ")")
        return
    seen = {}
    for inst in out.splitlines():
        inst = inst.strip()
        if "VID_16C0" not in inst:
            continue
        m = re.search(r"PID_([0-9A-Fa-f]{4})", inst)
        if not m:
            continue
        pid = int(m.group(1), 16)
        base = re.sub(r"&MI_..", "", inst).split("\\")[-1]   # collapse composite interfaces -> 1 device
        seen[(pid, base)] = pid
    if not seen:
        print("  none connected (plug the board in with a DATA usb cable)")
    for pid in sorted(set(seen.values())):
        print(f"  PID 0x{pid:04X}: {PJRC_PID.get(pid, 'unknown type')}")


def main():
    serial_scan()
    if sys.platform.startswith("win"):
        usb_scan_windows()
    else:
        print("\n(Full USB scan is Windows-only here; on Linux/Mac use `lsusb`/`ioreg` for VID 16c0.)")
    print("\nReading:")
    print("  - 0x048B Dual Serial  = our firmware is flashed (shows as 2 COM ports).")
    print("  - 0x0478 bootloader   = it's in flash mode.")
    print("  - 0x0486 / other HID  = factory program; flash our firmware to use it.")


if __name__ == "__main__":
    main()
