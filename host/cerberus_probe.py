#!/usr/bin/env python3
"""
Cerberus host probe -- cross-module IKA (DID 0x00BE) read over the Teensy USB serial.
Runs "Experiment 1": reads 0x00BE from J533/J255/J136/J285 and decides VIN-bound
vs module-bound from the AES-key bytes.

Flash Cerberus, wire Head 1 to OBD pins 6/14, then:
    pip install pyserial
    python cerberus_probe.py COM5
"""
import sys, time
import serial  # pip install pyserial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"

MODULES = [
    ("J533 Gateway",     0x710, 0x77A),
    ("J255 Climatronic", 0x746, 0x7B0),
    ("J136 Mem.Seat Drv",0x74C, 0x7B6),
    ("J285 Cluster",     0x714, 0x77E),
]
KNOWN_J136 = "E62B41D11C44AF202177FB1F274B0AC2D15BD262E4FD27AB61D123C2F15A2C932600"


def cmd(s, tx, rx, req_hex, settle=0.15):
    s.reset_input_buffer()
    s.write(("%03X:%03X:%s\n" % (tx, rx, req_hex)).encode())
    time.sleep(settle)
    return s.readline().decode(errors="replace").strip()


def main():
    s = serial.Serial(PORT, 115200, timeout=2)
    time.sleep(0.4)
    aes_keys = {}
    for name, tx, rx in MODULES:
        print("== %-18s TX %03X RX %03X ==" % (name, tx, rx))
        print("  10 03 :", cmd(s, tx, rx, "1003"))
        r = cmd(s, tx, rx, "2200BE", settle=0.4)
        print("  0x00BE:", r)
        if r.startswith("OK:"):
            blob = r[3:].upper()
            if blob.startswith("6200BE") and len(blob) >= 6 + 32:
                key = blob[6:6 + 32]
                print("      AES key:", key)
                if any(c != "0" for c in key):
                    aes_keys[name] = key
                if blob[6:].startswith(KNOWN_J136[:32]):
                    print("      *** matches J136 Feb-2024 key ***")
        print()

    print("=== VERDICT ===")
    if len(aes_keys) >= 2:
        if len(set(aes_keys.values())) == 1:
            print("  All AES keys IDENTICAL -> VIN-BOUND. Replay the known blob; no derivation.")
        else:
            print("  AES keys DIFFER -> MODULE-BOUND. Derivation needed.")
            for n, k in aes_keys.items():
                print("    %-18s %s" % (n, k))
    else:
        print("  Need >=2 non-zero blobs to decide; got %d." % len(aes_keys))


if __name__ == "__main__":
    main()
