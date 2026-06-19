"""
DoIP (ISO 13400-2) capture decoder for the Cerberus DoIP head.

Pipeline (see docs/PRODUCT-LINE.md "DoIP capture"):
    pcap stream  ->  peel Ethernet/IP/TCP-UDP  ->  filter port 13400  ->
    parse the 8-byte DoIP header + payload  ->  hand UDS payloads to the SAME
    UDS/SAE decode we already use for CAN (sae_decode.py).

DoIP is just transport: a "diagnostic message" (payload type 0x8001) carries a
source/target logical address + a raw UDS (ISO 14229) request/response. So the
diagnostic *meaning* reuses sae_decode (DTCs) — this module only adds the DoIP
framing + the UDS service/NRC names.

Pure stdlib on purpose (the Console ships pyserial-only — no dpkt/Scapy). IPv4 +
single-segment DoIP messages are handled; IPv6 and cross-segment TCP reassembly
are marked TODO (own-session diagnostic/flash captures rarely need them).

Status: SCAFFOLD. The DoIP head is unbuilt — this parser is offline-testable today
(`python doip_decode.py` runs a synthetic-pcap self-test) so it's ready when the
4.1 + Ethernet PHY exists.
"""
import os
import struct

try:                                  # reuse the CAN-side UDS DTC decoder if present
    import sae_decode
except Exception:                     # pragma: no cover - graceful when run standalone
    sae_decode = None

DOIP_PORT = 13400

# ---- ISO 13400-2 payload types -------------------------------------------------
PAYLOAD_TYPES = {
    0x0000: "Generic DoIP header negative acknowledge",
    0x0001: "Vehicle identification request",
    0x0002: "Vehicle identification request (EID)",
    0x0003: "Vehicle identification request (VIN)",
    0x0004: "Vehicle announcement / identification response",
    0x0005: "Routing activation request",
    0x0006: "Routing activation response",
    0x0007: "Alive check request",
    0x0008: "Alive check response",
    0x4001: "DoIP entity status request",
    0x4002: "DoIP entity status response",
    0x4003: "Diagnostic power mode information request",
    0x4004: "Diagnostic power mode information response",
    0x8001: "Diagnostic message",
    0x8002: "Diagnostic message positive acknowledge",
    0x8003: "Diagnostic message negative acknowledge",
}

# routing activation response code (single byte)
ROUTING_RESP = {
    0x00: "unknown source address",
    0x01: "all sockets registered/active",
    0x02: "SA different from already-activated SA",
    0x03: "SA already registered on a different socket",
    0x04: "missing authentication",
    0x05: "rejected confirmation",
    0x06: "unsupported routing activation type",
    0x10: "success",
    0x11: "requires confirmation",
}

# diagnostic message negative-ack code (single byte)
DIAG_NACK = {
    0x00: "invalid source address",
    0x01: "unknown target address",
    0x02: "diagnostic message too large",
    0x03: "out of memory",
    0x04: "target unreachable",
    0x05: "unknown network",
    0x06: "transport protocol error",
}

# ---- UDS (ISO 14229) service ids + negative-response codes ----------------------
UDS_SID = {
    0x10: "DiagnosticSessionControl", 0x11: "ECUReset", 0x14: "ClearDTC",
    0x19: "ReadDTCInformation", 0x22: "ReadDataByIdentifier",
    0x23: "ReadMemoryByAddress", 0x27: "SecurityAccess",
    0x28: "CommunicationControl", 0x2E: "WriteDataByIdentifier",
    0x2F: "InputOutputControl", 0x31: "RoutineControl",
    0x34: "RequestDownload", 0x35: "RequestUpload", 0x36: "TransferData",
    0x37: "RequestTransferExit", 0x3D: "WriteMemoryByAddress",
    0x3E: "TesterPresent", 0x85: "ControlDTCSetting",
}
UDS_NRC = {
    0x10: "general reject", 0x11: "service not supported",
    0x12: "sub-function not supported", 0x13: "incorrect length/format",
    0x22: "conditions not correct", 0x24: "request sequence error",
    0x31: "request out of range", 0x33: "security access denied",
    0x35: "invalid key", 0x36: "exceeded attempts", 0x37: "required time delay",
    0x78: "response pending", 0x7E: "sub-function not supported in session",
    0x7F: "service not supported in session",
}


# ================================================================================
# pcap container (classic libpcap; link-type EN10MB / Ethernet)
# ================================================================================
_PCAP_MAGIC = {0xA1B2C3D4: ">", 0xD4C3B2A1: "<",          # us-resolution
               0xA1B23C4D: ">", 0x4D3CB2A1: "<"}          # ns-resolution

def iter_pcap(data: bytes):
    """Yield (ts_seconds_float, frame_bytes) for each record in a libpcap blob."""
    if len(data) < 24:
        return
    magic = struct.unpack(">I", data[:4])[0]
    endian = _PCAP_MAGIC.get(magic)
    if endian is None:
        raise ValueError("not a libpcap file (bad magic)")
    ns = magic in (0xA1B23C4D, 0x4D3CB2A1)
    off = 24                                              # skip global header
    while off + 16 <= len(data):
        ts_sec, ts_frac, caplen, _orig = struct.unpack(endian + "IIII", data[off:off + 16])
        off += 16
        frame = data[off:off + caplen]
        off += caplen
        ts = ts_sec + ts_frac / (1e9 if ns else 1e6)
        yield ts, frame


def write_pcap(records) -> bytes:
    """Build a libpcap (EN10MB, us) blob from [(ts_float, frame_bytes), ...]."""
    out = bytearray(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 262144, 1))
    for ts, frame in records:
        sec = int(ts)
        usec = int(round((ts - sec) * 1e6))
        out += struct.pack("<IIII", sec, usec, len(frame), len(frame)) + frame
    return bytes(out)


# ================================================================================
# L2-L4 peel  ->  (src_port, dst_port, l4_proto, payload)   or None
# ================================================================================
def peel(frame: bytes):
    if len(frame) < 14:
        return None
    etype = struct.unpack(">H", frame[12:14])[0]
    off = 14
    while etype == 0x8100 and len(frame) >= off + 4:     # VLAN tag(s)
        etype = struct.unpack(">H", frame[off + 2:off + 4])[0]
        off += 4
    if etype != 0x0800:                                  # IPv4 only (TODO: 0x86DD IPv6)
        return None
    if len(frame) < off + 20:
        return None
    ihl = (frame[off] & 0x0F) * 4
    proto = frame[off + 9]
    l4 = off + ihl
    if proto == 6:                                       # TCP
        if len(frame) < l4 + 20:
            return None
        sport, dport = struct.unpack(">HH", frame[l4:l4 + 4])
        doff = (frame[l4 + 12] >> 4) * 4
        return sport, dport, "TCP", frame[l4 + doff:]
    if proto == 17:                                      # UDP
        if len(frame) < l4 + 8:
            return None
        sport, dport = struct.unpack(">HH", frame[l4:l4 + 4])
        return sport, dport, "UDP", frame[l4 + 8:]
    return None


# ================================================================================
# DoIP header + payload
# ================================================================================
def iter_doip(payload: bytes):
    """Yield parsed DoIP messages from an L4 payload (may hold several back-to-back)."""
    off = 0
    while off + 8 <= len(payload):
        ver, inv, ptype, plen = struct.unpack(">BBHI", payload[off:off + 8])
        body = payload[off + 8:off + 8 + plen]
        if len(body) < plen:                             # truncated / spans segments (TODO reassembly)
            yield {"version": ver, "type": ptype, "len": plen, "body": body, "truncated": True}
            return
        yield _decode_payload(ver, ptype, body)
        off += 8 + plen


def _decode_payload(ver, ptype, body):
    msg = {"version": ver, "type": ptype, "type_name": PAYLOAD_TYPES.get(ptype, f"0x{ptype:04X} (unknown)"),
           "len": len(body), "body": body}
    if ptype == 0x0006 and len(body) >= 5:               # routing activation response
        msg["client_addr"] = struct.unpack(">H", body[0:2])[0]
        msg["entity_addr"] = struct.unpack(">H", body[2:4])[0]
        msg["resp_code"] = body[4]
        msg["resp_text"] = ROUTING_RESP.get(body[4], "reserved/OEM")
    elif ptype in (0x0004,) and len(body) >= 19:         # vehicle announcement: VIN(17)+LA(2)+...
        msg["vin"] = body[0:17].decode("latin-1", "replace")
        msg["logical_addr"] = struct.unpack(">H", body[17:19])[0]
    elif ptype == 0x8001 and len(body) >= 4:             # diagnostic message (carries UDS)
        msg["source"] = struct.unpack(">H", body[0:2])[0]
        msg["target"] = struct.unpack(">H", body[2:4])[0]
        msg["uds"] = body[4:]
        msg["uds_text"] = decode_uds(body[4:])
    elif ptype == 0x8002 and len(body) >= 5:             # diag positive ack
        msg["source"] = struct.unpack(">H", body[0:2])[0]
        msg["target"] = struct.unpack(">H", body[2:4])[0]
        msg["ack_code"] = body[4]
    elif ptype == 0x8003 and len(body) >= 5:             # diag negative ack
        msg["source"] = struct.unpack(">H", body[0:2])[0]
        msg["target"] = struct.unpack(">H", body[2:4])[0]
        msg["nack_code"] = body[4]
        msg["nack_text"] = DIAG_NACK.get(body[4], "reserved/OEM")
    return msg


def decode_uds(uds: bytes) -> str:
    """One-line summary of a UDS request/response carried in a DoIP diagnostic message."""
    if not uds:
        return ""
    sid = uds[0]
    if sid == 0x7F and len(uds) >= 3:                    # negative response
        svc = UDS_SID.get(uds[1], f"0x{uds[1]:02X}")
        return f"NegativeResponse to {svc}: {UDS_NRC.get(uds[2], f'NRC 0x{uds[2]:02X}')}"
    if sid & 0x40:                                       # positive response (req SID | 0x40)
        name = UDS_SID.get(sid & ~0x40, f"0x{sid & 0x7F:02X}")
        extra = _uds_detail(sid & ~0x40, uds[1:], resp=True)
        return f"{name} response{(' — ' + extra) if extra else ''}"
    name = UDS_SID.get(sid, f"0x{sid:02X}")
    extra = _uds_detail(sid, uds[1:], resp=False)
    return f"{name} request{(' — ' + extra) if extra else ''}"


def _uds_detail(svc, data, resp):
    if svc == 0x22 and len(data) >= 2:                   # ReadDataByIdentifier
        return f"DID 0x{struct.unpack('>H', data[0:2])[0]:04X}"
    if svc == 0x19 and resp and sae_decode and len(data) >= 4:   # ReadDTCInformation response
        # data: [reportType][...status/DTC records]; decode the first DTC if present
        recs = data[1:]
        if len(recs) >= 4:
            dtc = recs[0:3]
            ftb = recs[2]
            code = _dtc_to_sae(dtc)
            txt = sae_decode.describe(code, ftb)
            return f"{code}-{ftb:02X}" + (f"  {txt}" if txt else "")
    return ""


def _dtc_to_sae(b3: bytes) -> str:
    """3-byte UDS DTC high bytes -> SAE code letter+digits (same mapping as the CAN path)."""
    first = b3[0]
    letter = "PCBU"[(first >> 6) & 0x3]
    return f"{letter}{(first >> 4) & 0x3}{first & 0xF:X}{b3[1]:02X}"[:5]


# ================================================================================
# top-level: pcap blob -> human-readable lines
# ================================================================================
def summarize_pcap(data: bytes):
    """Yield 'ts  dir  type  detail' lines for every DoIP message on port 13400."""
    for ts, frame in iter_pcap(data):
        peeled = peel(frame)
        if not peeled:
            continue
        sport, dport, proto, payload = peeled
        if DOIP_PORT not in (sport, dport):
            continue
        for m in iter_doip(payload):
            detail = (m.get("uds_text") or m.get("resp_text") or m.get("nack_text")
                      or m.get("vin") or "")
            yield f"{ts:14.6f}  {proto}  {m['type_name']}" + (f"  | {detail}" if detail else "")


# ================================================================================
# self-test (offline; no hardware) — build a synthetic pcap and parse it
# ================================================================================
def _eth_ip_tcp(payload, sport=13400, dport=49152):
    eth = b"\x02\x00\x00\x00\x00\x01" + b"\x02\x00\x00\x00\x00\x02" + b"\x08\x00"
    tcp = struct.pack(">HHIIBBHHH", sport, dport, 0, 0, 0x50, 0x18, 0xFFFF, 0, 0)
    total = 20 + len(tcp) + len(payload)
    ip = struct.pack(">BBHHHBBH", 0x45, 0, total, 0, 0x4000, 64, 6, 0) + b"\x0a\x00\x00\x01\x0a\x00\x00\x02"
    return eth + ip + tcp + payload


def _doip(ptype, body):
    return struct.pack(">BBHI", 0x02, 0xFD, ptype, len(body)) + body


def _selftest():
    # 1) routing activation response: client 0x0E80, entity 0x1000, success
    ra = _doip(0x0006, struct.pack(">HHB", 0x0E80, 0x1000, 0x10) + b"\x00\x00\x00\x00")
    # 2) diagnostic message: ECU->tester, UDS positive ReadDataByIdentifier(F190 VIN)
    vin = b"WAUGGAFC7DN120188"
    uds = b"\x62\xF1\x90" + vin
    dm = _doip(0x8001, struct.pack(">HH", 0x1000, 0x0E80) + uds)
    # 3) negative response: SecurityAccess denied
    dmn = _doip(0x8001, struct.pack(">HH", 0x1000, 0x0E80) + b"\x7F\x27\x33")
    blob = write_pcap([
        (1.000000, _eth_ip_tcp(ra)),
        (1.250000, _eth_ip_tcp(dm, sport=13400, dport=49152)),
        (1.500000, _eth_ip_tcp(dmn)),
    ])
    lines = list(summarize_pcap(blob))
    for ln in lines:
        print(ln)
    assert len(lines) == 3, f"expected 3 DoIP msgs, got {len(lines)}"
    assert "Routing activation response" in lines[0] and "success" in lines[0]
    assert "Diagnostic message" in lines[1] and "ReadDataByIdentifier response" in lines[1]
    assert "DID 0xF190" in lines[1]
    assert "NegativeResponse to SecurityAccess" in lines[2] and "security access denied" in lines[2]
    # round-trip the pcap container itself
    assert sum(1 for _ in iter_pcap(blob)) == 3
    print("\ndoip_decode self-test OK")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:                                 # summarize a real capture
        with open(sys.argv[1], "rb") as fh:
            blob = fh.read()
        n = 0
        for ln in summarize_pcap(blob):
            print(ln)
            n += 1
        print(f"\n{n} DoIP message(s) on port {DOIP_PORT}")
    else:
        _selftest()
