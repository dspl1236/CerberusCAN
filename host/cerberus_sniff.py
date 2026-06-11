#!/usr/bin/env python3
"""
Cerberus host sniffer -- passive capture of a CAN bus via the Teensy SNIFF command.
Aimed at catching the J533<->J255 Component-Protection handshake on the comfort bus
(Head 2), but works on either head.

Flash Cerberus, wire the head you want to watch, then:
    pip install pyserial
    python cerberus_sniff.py COM5             # Head 2 (comfort), 15 s
    python cerberus_sniff.py COM5 2 30        # Head 2, 30 s
    python cerberus_sniff.py COM5 1 10 cap    # Head 1, 10 s, also write cap.csv

While it runs, trigger the event you want to capture (ignition cycle, module wake,
door, key, etc.). Frames stream live; a per-ID summary prints at the end.

Protocol:  host sends  SNIFF:<bus>:<ms>   ->   RX:<ms>:<id>:<data> ...  DONE:<n>

Note: at 115200 baud the USB link can lag a *flooded* bus; a CP handshake is a short
burst, so that's fine here. If you ever drop frames on a busy bus, shorten the window.
"""
import sys, time
import serial  # pip install pyserial


def summarize(frames):
    print("\n================ summary ================")
    if not frames:
        print("0 frames captured.")
        print("If the car was awake and you watched the COMFORT bus (Head 2), that")
        print("strongly suggests it's LOW-SPEED FAULT-TOLERANT -- the SN65HVD230 can't")
        print("read it, and that head would need a TJA1055T/3. (Head 1 @500k is unaffected.)")
        return
    by_id = {}
    for t, cid, data in frames:
        e = by_id.setdefault(cid, {"n": 0, "first": t, "last": t, "payloads": set(), "sample": data})
        e["n"] += 1
        e["last"] = t
        e["payloads"].add(data)
    print("%-5s %6s %9s %9s %6s  %s" % ("ID", "count", "first ms", "last ms", "uniq", "sample"))
    for cid in sorted(by_id, key=lambda c: by_id[c]["first"]):
        e = by_id[cid]
        print("%-5s %6d %9d %9d %6d  %s" % (cid, e["n"], e["first"], e["last"], len(e["payloads"]), e["sample"]))
    print("\n%d frames, %d distinct IDs." % (len(frames), len(by_id)))
    print("IDs whose payload changes (uniq>1) around your trigger are the ones to chase")
    print("for a challenge/response handshake.")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    bus  = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
    outpath = sys.argv[4] if len(sys.argv) > 4 else None
    if outpath and not outpath.lower().endswith(".csv"):
        outpath += ".csv"

    s = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()

    # link check
    s.write(b"PING\n")
    if "PONG" not in s.readline().decode(errors="replace"):
        print("warning: no PONG from Cerberus -- check the port/flash. Continuing anyway.")
    s.reset_input_buffer()

    print("Sniffing Head %d for %.0f s -- trigger your event now...\n" % (bus, secs))
    s.write(("SNIFF:%d:%d\n" % (bus, int(secs * 1000))).encode())

    frames = []
    out = open(outpath, "w") if outpath else None
    if out:
        out.write("ms,id,data\n")
    deadline = time.time() + secs + 5  # grace window for DONE
    try:
        while time.time() < deadline:
            line = s.readline().decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("RX:"):
                p = line.split(":")
                if len(p) >= 4:
                    t, cid, data = int(p[1]), p[2].upper(), p[3].upper()
                    frames.append((t, cid, data))
                    print("  %9d ms   %-3s   %s" % (t, cid, data))
                    if out:
                        out.write("%d,%s,%s\n" % (t, cid, data))
            elif line.startswith("DONE"):
                print("\n[%s]" % line)
                break
            elif line.startswith("ERR"):
                print("[error] %s" % line)
                break
    except KeyboardInterrupt:
        print("\n(interrupted)")
    finally:
        if out:
            out.close()
            print("raw frames -> %s" % outpath)
        s.close()
    summarize(frames)


if __name__ == "__main__":
    main()
