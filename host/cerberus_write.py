#!/usr/bin/env python3
"""
Cerberus host writer -- a careful, GENERIC UDS write driver over the Teensy USB serial.

Flow:  read-before  ->  show plan  ->  confirm  ->  (10 03 extended session)  ->
       2E write  ->  read-back verify.

This is NOT a Component-Protection recipe. You supply the target ID, DID, and data,
so point it wherever Experiment 1's verdict tells you. Multi-frame writes (e.g. the
34-byte IKA blob at 0x00BE) are ISO-TP framed automatically by the firmware.

    pip install pyserial

    # DRY RUN: read current value + show exactly what WOULD be sent, send nothing
    python cerberus_write.py COM5 --tx 710 --rx 77A --did 00BE --data <68hex> --dry-run

    # real write (prompts for 'yes'; --yes to skip the prompt)
    python cerberus_write.py COM5 --tx 710 --rx 77A --did 04A3 --data <20hex>

VAG CP reference (use ONLY after the read says what to write, and to which module):
    IKA blob       DID 00BE  34 bytes (68 hex)
    constellation  DID 04A3  10 bytes (20 hex)
"""
import sys, time, argparse
import serial  # pip install pyserial

NRC = {
    0x11: "serviceNotSupported", 0x13: "incorrectLength/Format",
    0x22: "conditionsNotCorrect", 0x31: "requestOutOfRange",
    0x33: "securityAccessDenied", 0x35: "invalidKey", 0x72: "generalProgrammingFailure",
    0x7E: "subFunctionNotSupportedInActiveSession",
    0x7F: "serviceNotSupportedInActiveSession",
}


def is_hex_bytes(s):
    try:
        int(s, 16)
    except ValueError:
        return False
    return len(s) > 0 and len(s) % 2 == 0


def txn(s, tx, rx, req_hex):
    s.reset_input_buffer()
    line = "%03X:%03X:%s" % (tx, rx, req_hex.upper())
    s.write((line + "\n").encode())
    resp = s.readline().decode(errors="replace").strip()
    print("   >> %s" % line)
    print("   << %s" % (resp or "(no reply / timeout)"))
    return resp


def decode_neg(resp, service):
    """If resp carries a UDS negative response (7F ss nrc), return a reason string."""
    body = resp[3:].upper() if resp.startswith("OK:") else ""
    if body.startswith("7F") and len(body) >= 6:
        nrc = int(body[4:6], 16)
        return "NEGATIVE 7F %02X %02X (%s)" % (service, nrc, NRC.get(nrc, "NRC 0x%02X" % nrc))
    return None


def main():
    ap = argparse.ArgumentParser(description="Cerberus generic UDS write driver (read-before, verify-after).")
    ap.add_argument("port")
    ap.add_argument("--tx", required=True, help="request CAN id, hex (e.g. 710)")
    ap.add_argument("--rx", required=True, help="response CAN id, hex (e.g. 77A)")
    ap.add_argument("--did", required=True, help="2-byte data identifier, hex (e.g. 00BE)")
    ap.add_argument("--data", required=True, help="bytes to write, hex (even length)")
    ap.add_argument("--session", default="1003", help="session to enter first, hex (default 1003; 'none' to skip)")
    ap.add_argument("--dry-run", action="store_true", help="read current value + show plan, send NOTHING")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    a = ap.parse_args()

    tx, rx = int(a.tx, 16), int(a.rx, 16)
    try:
        int(a.did, 16)
    except ValueError:
        sys.exit("error: --did must be hex, e.g. 00BE")
    did = a.did.upper().zfill(4)
    if len(did) != 4:
        sys.exit("error: --did must be a 2-byte identifier, e.g. 00BE")
    data = a.data.upper()
    if not is_hex_bytes(data):
        sys.exit("error: --data must be an even number of hex digits")

    s = serial.Serial(a.port, 115200, timeout=3)
    time.sleep(0.3)
    s.reset_input_buffer()

    s.write(b"INFO\n")
    print("link: %s\n" % (s.readline().decode(errors="replace").strip() or "(no reply -- check port/flash)"))

    # 1) read BEFORE
    print("[1] read-before  DID %s" % did)
    before = txn(s, tx, rx, "22" + did)
    cur = before[9:].upper() if before.startswith("OK:62" + did) else ""
    print("    current value: %s" % (cur or "(unreadable -- continuing, verify by hand)"))
    print()

    # 2) plan
    write_req = "2E" + did + data
    print("[2] plan")
    print("    target   TX %03X  RX %03X" % (tx, rx))
    print("    DID      %s" % did)
    print("    new data %s  (%d bytes)" % (data, len(data) // 2))
    if cur:
        print("    current  %s" % cur)
        if cur == data:
            print("    NOTE: new data == current value; nothing would change.")
    print("    will send (after session): %s" % write_req)
    print()

    if a.dry_run:
        print("DRY RUN -- nothing was written. Re-run without --dry-run to commit.")
        s.close(); return

    # 3) confirm
    if not a.yes:
        print("About to WRITE to a live module. This changes stored data.")
        if input("Type 'yes' to proceed: ").strip().lower() != "yes":
            print("aborted."); s.close(); return
    print()

    # 4) extended session
    if a.session.lower() != "none":
        print("[3] enter session %s" % a.session.upper())
        sr = txn(s, tx, rx, a.session)
        neg = decode_neg(sr, int(a.session[:2], 16))
        if neg or not sr.startswith("OK:"):
            print("    session failed (%s) -- aborting before write." % (neg or sr))
            s.close(); return
        print()

    # 5) write
    print("[4] write  %s" % write_req)
    wr = txn(s, tx, rx, write_req)
    if wr.startswith("ERR"):
        print("    transport error (%s) -- not written/confirmed." % wr); s.close(); return
    neg = decode_neg(wr, 0x2E)
    if neg:
        print("    write rejected: %s" % neg); s.close(); return
    if not wr.startswith("OK:6E" + did):
        print("    unexpected write response -- NOT verified."); s.close(); return
    print("    write accepted (6E %s)" % did)
    print()

    # 6) read-back verify
    print("[5] read-back verify  DID %s" % did)
    rb = txn(s, tx, rx, "22" + did)
    if rb.startswith("OK:62" + did):
        got = rb[9:].upper()
        if got == data:
            print("    *** VERIFIED: stored value matches what we wrote. ***")
        else:
            print("    !!! MISMATCH:")
            print("        wrote: %s" % data)
            print("        read : %s" % got)
    else:
        print("    read-back failed -- could not confirm.")
    s.close()


if __name__ == "__main__":
    main()
