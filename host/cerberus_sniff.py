#!/usr/bin/env python3
"""
Cerberus host sniffer -- passive CAN capture with a LIVE view + simultaneous save.

Streams frames from the Teensy SNIFF command, shows a live running tally so you can
confirm data is actually flowing, and writes every frame to CSV as it arrives.

Usage:
    pip install pyserial
    python cerberus_sniff.py COM5 1 0  cpcap      # Head 1 (500k), run until Ctrl-C, save cpcap.csv
    python cerberus_sniff.py COM5 1 30 cpcap      # Head 1, 30 s window, save cpcap.csv
    python cerberus_sniff.py COM5 1 0  cpcap -v   # also stream every frame (firehose)

  bus:  1 = Head 1 / Powertrain-Diagnostic CAN (OBD 6+14, 500k)  <- the one that works,
            and where ODIS routes ALL its diagnostics (incl. the CP handshake)
        2 = Head 2 / Comfort CAN (OBD 3+11, 100k)  [needs an FT transceiver; HVD230 can't read LS-FT]
  secs: 0 = run until you press Ctrl-C  (recommended for a full ODIS session)

Live display (default): a single refreshing status line ->
    12.3s |   4821 frames |  37 IDs |   391 f/s | last 6C8 10 0E 62 ...
The frame count climbing = data is being seen. Use -v to also scroll every frame.

Protocol:  host -> SNIFF:<bus>:<ms>   ;   device -> RX:<ms>:<id>:<data> ... DONE:<n>
(ms=0 = run until any serial byte arrives; Ctrl-C sends that byte to stop cleanly.)
"""
import sys
import time

try:
    import serial  # pip install pyserial
except ImportError:
    sys.exit("pyserial not installed -- run:  pip install pyserial")


def summarize(frames, bus):
    print("\n================ summary ================")
    if not frames:
        print("0 frames captured.")
        if bus == 2:
            print("Head 2 (comfort) silent with the car awake almost always means the bus")
            print("is LOW-SPEED FAULT-TOLERANT -- the SN65HVD230 can't read it; that head")
            print("needs a TJA1055T/3. Capture Head 1 (500k) instead -- ODIS routes the CP")
            print("handshake there anyway.")
        else:
            print("No traffic on Head 1. Check: ignition on, wiring to OBD pins 6(CANH)/14(CANL),")
            print("and that the splitter actually carries the diag CAN.")
        return
    by_id = {}
    for t, cid, data in frames:
        e = by_id.setdefault(cid, {"n": 0, "first": t, "last": t, "uniq": set()})
        e["n"] += 1
        e["last"] = t
        e["uniq"].add(data)
    print("%-5s %7s %9s %9s %6s" % ("ID", "count", "first ms", "last ms", "uniq"))
    for cid in sorted(by_id, key=lambda c: by_id[c]["first"]):
        e = by_id[cid]
        print("%-5s %7d %9d %9d %6d" % (cid, e["n"], e["first"], e["last"], len(e["uniq"])))
    print("\n%d frames, %d distinct IDs." % (len(frames), len(by_id)))
    print("IDs whose payload changes (uniq>1) around your trigger are the ones to chase")
    print("for a challenge/response -- then run the TP 2.0 / ISO-TP decoder on the CSV.")


def main():
    verbose = ("-v" in sys.argv) or ("--verbose" in sys.argv)
    argv = [a for a in sys.argv[1:] if a not in ("-v", "--verbose")]
    port = argv[0] if len(argv) > 0 else "COM5"
    bus = int(argv[1]) if len(argv) > 1 else 1
    secs = float(argv[2]) if len(argv) > 2 else 0.0
    outpath = argv[3] if len(argv) > 3 else None
    if outpath and not outpath.lower().endswith(".csv"):
        outpath += ".csv"

    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()

    s.write(b"PING\n")
    if b"PONG" not in s.readline():
        print("warning: no PONG from Cerberus -- check the COM port / firmware. Continuing.")
    s.reset_input_buffer()

    ms = 0 if secs <= 0 else int(secs * 1000)
    print("Sniffing Head %d (%s). Trigger your event now.  [Ctrl-C to stop]\n"
          % (bus, "until Ctrl-C" if ms == 0 else "%.0f s" % secs))
    s.write(("SNIFF:%d:%d\n" % (bus, ms)).encode())

    out = open(outpath, "w", buffering=1) if outpath else None   # line-buffered = flush per frame
    if out:
        out.write("ms,id,data\n")

    frames = []
    ids = set()
    n = 0
    t0 = time.time()
    last = ""
    last_draw = 0.0
    buf = b""
    stop_deadline = None

    def draw():
        el = time.time() - t0
        rate = n / el if el > 0 else 0.0
        sys.stdout.write("\r  %6.1fs | %7d frames | %3d IDs | %6.0f f/s | last %-26s"
                         % (el, n, len(ids), rate, last))
        sys.stdout.flush()

    running = True
    while running:
        try:
            if ms and (time.time() - t0) > secs + 5:
                break
            if stop_deadline and time.time() > stop_deadline:
                break
            chunk = s.read(s.in_waiting or 1)
            if chunk:
                buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line.startswith("RX:"):
                    p = line.split(":")
                    if len(p) >= 4:
                        t, cid, data = int(p[1]), p[2].upper(), p[3].upper()
                        frames.append((t, cid, data))
                        ids.add(cid)
                        n += 1
                        last = cid + "  " + data
                        if out:
                            out.write("%d,%s,%s\n" % (t, cid, data))
                        if verbose:
                            print("  %9d ms  %-3s  %s" % (t, cid, data))
                elif line.startswith("DONE"):
                    running = False
                    break
                elif line.startswith("ERR"):
                    sys.stdout.write("\n[error] %s\n" % line)
                    running = False
                    break
            if not verbose and (time.time() - last_draw) > 0.2:
                draw()
                last_draw = time.time()
        except KeyboardInterrupt:
            if stop_deadline is None:
                try:
                    s.write(b"\n")          # ms=0 sniff stops on any serial byte
                except Exception:
                    pass
                stop_deadline = time.time() + 1.5   # short drain to catch trailing frames + DONE
            else:
                break

    try:
        if out:
            out.close()
    finally:
        s.close()
    sys.stdout.write("\n")
    if outpath:
        print("saved %d frames -> %s" % (n, outpath))
    summarize(frames, bus)


if __name__ == "__main__":
    main()
