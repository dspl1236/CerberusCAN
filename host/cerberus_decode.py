#!/usr/bin/env python3
"""
cerberus_decode.py -- turn a cerberus_sniff CSV into labeled UDS/KWP exchanges.

Reassembles BOTH transports seen on the 500k diag CAN:
  * ISO-TP (ISO 15765-2)  -- the 0x7xx UDS request/response pairs
  * VW TP 2.0             -- KWP modules routed by the gateway (where TrainICA rides)

and prints a timeline, flagging the CP-relevant services (TrainICA/GVA, the 0x00BE
IKA write, SecurityAccess) and any 34-byte blob that looks like an IKA.

Usage:
    python cerberus_decode.py cpcap.csv
"""
import sys

UDS = {
    0x10: "DiagSession", 0x50: "DiagSession+",
    0x27: "SecurityAccess", 0x67: "SecurityAccess+",
    0x22: "ReadDID", 0x62: "ReadDID+",
    0x2E: "WriteDID", 0x6E: "WriteDID+",
    0x2F: "IOControl", 0x6F: "IOControl+",
    0x31: "Routine", 0x71: "Routine+",
    0x3E: "TesterPresent", 0x7E: "TesterPresent+",
    0x14: "ClearDTC", 0x54: "ClearDTC+",
    0x19: "ReadDTC", 0x59: "ReadDTC+",
    0x1A: "ReadECUId(KWP)", 0x5A: "ReadECUId+",
    0x21: "ReadLocalID(KWP)", 0x61: "ReadLocalID+",
    0x3B: "WriteLocalID(KWP)", 0x7B: "WriteLocalID+",
    0x23: "ReadMem", 0x63: "ReadMem+",
    0x34: "RequestDownload", 0x74: "RequestDownload+",
    0x36: "TransferData", 0x76: "TransferData+",
    0x37: "ReqXferExit", 0x77: "ReqXferExit+",
    0x7F: "NegativeResponse",
}
NRC = {0x11: "serviceNotSupported", 0x12: "subFuncNotSupported", 0x13: "wrongLength",
       0x22: "conditionsNotCorrect", 0x24: "requestSequenceError", 0x31: "requestOutOfRange",
       0x33: "securityAccessDenied", 0x35: "invalidKey", 0x36: "exceedAttempts",
       0x78: "responsePending", 0x7F: "serviceNotSupportedInSession"}


def ascii_of(b):
    return "".join(chr(c) if 32 <= c < 127 else "." for c in b)


def label(payload):
    if not payload:
        return ""
    sid = payload[0]
    name = UDS.get(sid, "svc:0x%02X" % sid)
    extra = ""
    flag = ""
    if sid == 0x7F and len(payload) >= 3:
        extra = " (%s, NRC=0x%02X %s)" % (UDS.get(payload[1], "0x%02X" % payload[1]),
                                          payload[2], NRC.get(payload[2], "?"))
    elif sid in (0x22, 0x62, 0x2E, 0x6E) and len(payload) >= 3:
        extra = " DID=%02X%02X" % (payload[1], payload[2])
        if sid == 0x2E and payload[1] == 0x00 and payload[2] == 0xBE:
            flag = "   <<<<< IKA WRITE (0x00BE)"
    elif sid in (0x21, 0x61, 0x3B, 0x7B) and len(payload) >= 2:
        extra = " LID=%02X" % payload[1]
        if sid == 0x3B:
            flag = "   <<<<< KWP WRITE (TrainICA/GVA candidate)"
    elif sid in (0x27, 0x67) and len(payload) >= 2:
        extra = " level=%02X" % payload[1]
        flag = "   <<<<< SecurityAccess"
    if len(payload) in (34, 35):
        flag += "   [34B blob -> possible IKA]"
    return name + extra + flag


def isotp_reassemble(frames):
    """frames: list of (t, payload_bytes) for ONE can id. Yields (t, message_bytes)."""
    buf = bytearray()
    need = 0
    for t, d in frames:
        if not d:
            continue
        pci = d[0] >> 4
        if pci == 0:                                  # single frame
            n = d[0] & 0x0F
            yield (t, bytes(d[1:1 + n]))
            buf, need = bytearray(), 0
        elif pci == 1:                                # first frame
            need = ((d[0] & 0x0F) << 8) | d[1]
            buf = bytearray(d[2:])
        elif pci == 2:                                # consecutive frame
            if need:
                buf += d[1:]
                if len(buf) >= need:
                    yield (t, bytes(buf[:need]))
                    buf, need = bytearray(), 0
        # pci 3 = flow control -> ignore


def tp20_reassemble(frames):
    """VW TP 2.0 data frames for ONE channel id. Length-based reassembly.
    frames: list of (t, payload_bytes). Yields (t, message_bytes)."""
    buf = bytearray()
    need = 0
    for t, d in frames:
        if not d:
            continue
        op = d[0] >> 4
        if op in (0xA, 0xB):                           # connection setup / ACK -> not data
            continue
        body = d[1:]
        if need == 0:                                  # first frame of a message
            if len(body) < 2:
                continue
            need = (body[0] << 8) | body[1]            # 2-byte big-endian KWP length
            buf = bytearray(body[2:])
        else:
            buf += body
        if need and len(buf) >= need:
            yield (t, bytes(buf[:need]))
            buf, need = bytearray(), 0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "cpcap.csv"
    rows = []
    with open(path) as f:
        next(f, None)  # header
        for line in f:
            p = line.strip().split(",")
            if len(p) >= 3 and p[0].isdigit():
                t = int(p[0]); cid = int(p[1], 16)
                data = bytes.fromhex(p[2].replace(" ", "")) if p[2] else b""
                rows.append((t, cid, data))
    print("loaded %d frames from %s\n" % (len(rows), path))

    # --- discover VW TP 2.0 channels from setup (0x200 req C0 / 0x2xx resp D0) ---
    tp_ids = set()
    for t, cid, d in rows:
        if cid == 0x200 and len(d) >= 6 and d[1] == 0xC0:
            a = (d[3] << 8) | d[2]; b = (d[5] << 8) | d[4]
            tp_ids.update({a & 0x7FF, b & 0x7FF})
        if 0x200 < cid <= 0x2FF and len(d) >= 6 and d[1] == 0xD0:
            a = (d[3] << 8) | d[2]; b = (d[5] << 8) | d[4]
            tp_ids.update({a & 0x7FF, b & 0x7FF})
    tp_ids.discard(0)
    if tp_ids:
        print("VW TP 2.0 channel data IDs: %s\n" % ", ".join("0x%03X" % i for i in sorted(tp_ids)))

    # --- collect per-id frame lists ---
    by_id = {}
    for t, cid, d in rows:
        by_id.setdefault(cid, []).append((t, d))

    msgs = []  # (t, cid, transport, payload)
    for cid, fr in by_id.items():
        if cid in tp_ids:
            for t, m in tp20_reassemble(fr):
                msgs.append((t, cid, "TP20", m))
        elif 0x600 <= cid <= 0x7FF:                   # UDS diag range -> ISO-TP
            for t, m in isotp_reassemble(fr):
                msgs.append((t, cid, "ISO", m))
    msgs.sort(key=lambda x: x[0])

    print("%-9s %-5s %-5s  %-46s %s" % ("ms", "id", "tp", "decoded", "ascii"))
    print("-" * 100)
    for t, cid, tp, m in msgs:
        if m[:1] == b"\x3E" or m[:1] == b"\x7E":      # skip TesterPresent spam
            continue
        lab = label(m)
        asc = ascii_of(m) if any(32 <= c < 127 for c in m) else ""
        print("%-9d %-5s %-5s  %-46s %s" % (t, "%03X" % cid, tp, lab, asc[:40]))
        if "<<<<<" in lab:
            print("        full payload: %s" % m.hex())


if __name__ == "__main__":
    main()
