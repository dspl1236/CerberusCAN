#!/usr/bin/env python3
"""
tc1796_bsl.py -- drive the Infineon TC1796 (TriCore AudoNG) CAN Bootstrap Loader
from a CerberusCAN (Teensy) interface, for BENCH read/write of Continental
Simos 8 / 8.4 / 8.5 ECUs (e.g. Audi C7 3.0T 4G0907551x).

This is a clean reimplementation of the read/write ("Tier 1") path of
fastboatster's TC1796_CAN_BSL (https://github.com/fastboatster/TC1796_CAN_BSL,
GPL-3.0), adapted from SocketCAN to Cerberus's serial `CANX` primitive so the
whole protocol runs over one Teensy instead of a Pi + CAN hat. The reverse
engineering is bri3d's (TC1791_CAN_BSL) and fastboatster's (TC1796 port);
all credit to them.

SCOPE (Tier 1 -- what this file does):
  * upload `bootloader.bin` into ECU SPRAM via the Mask-ROM CAN BSL
  * unlock flash with the 8-byte read / 8-byte write passwords you already have
  * read arbitrary 32-bit words and small regions; erase; (write = TODO/v2)
NOT here (Tier 2): the timing-critical SBOOT password *extraction* (PWM/RST +
twister/crchack). That needs the firmware PWM/RST additions -- see
docs/tc1796-bsl-cerberus-port.md.

PREREQS:
  * CerberusCAN firmware >= 0.5 (provides the `CANX` command)
  * `bootloader.bin` from fastboatster/TC1796_CAN_BSL (pass with --bin)
  * ECU on the bench, in CAN-BSL mode (HWCFG strapped, CAN-transmitter woken).
    See the doc for wiring. Bus 1 (Head 1) @ 500k carries the BSL.

USAGE:
  python tc1796_bsl.py --selftest                       # offline framing check, no hardware
  python tc1796_bsl.py COM12 ping
  python tc1796_bsl.py COM12 upload --bin bootloader.bin
  python tc1796_bsl.py COM12 readid
  python tc1796_bsl.py COM12 read A0040000              # one 32-bit word
  python tc1796_bsl.py COM12 unlock-read ABF425508513C273
  python tc1796_bsl.py COM12 readmem A0040000 100 out.bin   # small region (slow, reliable)
"""
import sys
import time

BUS = 1                    # Head 1 (500k) carries the BSL
ID_INIT   = 0x100          # BSL init frame id
ID_ACK    = 0x40           # BSL init ACK comes back here (0x100 >> 2)
ID_DATA   = 0xC0           # data + RAM-bootloader command channel (0x300 >> 2)
BSL_SUCCESS = 0x55


# ----------------------------------------------------------------------------
# Frame construction (pure -- unit-tested by --selftest, no hardware needed)
# ----------------------------------------------------------------------------
def xor_chksum15(frame1: bytes, frame2_first7: bytes) -> int:
    """RAM-bootloader trailer checksum = XOR of the 15 header bytes, low byte."""
    x = 0
    for b in (frame1 + frame2_first7)[:15]:
        x ^= b
    return x & 0xFF


def init_frame(bootloader_len: int) -> bytes:
    """BSL init: 55 55 00 01 <nblocks LE16> 00 03   (ACK id 0x40, data id 0xC0)."""
    nblocks = (bootloader_len + 7) // 8           # ceil(len/8)
    return bytes([0x55, 0x55, 0x00, 0x01,
                  nblocks & 0xFF, (nblocks >> 8) & 0xFF,
                  0x00, 0x03])


def read_frames(addr: int):
    """read_byte_simos8: cmd 00 08, addr big-endian, + XOR-checksum trailer frame."""
    a = addr.to_bytes(4, "big")
    f1 = bytes([0x00, 0x08]) + a + bytes([0x00, 0x00])
    f2_7 = bytes(7)
    f2 = f2_7 + bytes([xor_chksum15(f1, f2_7)])
    return f1, f2


def password_frames(pw1: bytes, pw2: bytes, read_write: int = 0x00, ucb: int = 0x00):
    """simos8_send_passwords: cmd 00 10, pw1(4)+pw2(4). read_write 0x00=read, 0x01=write."""
    assert len(pw1) == 4 and len(pw2) == 4
    f1 = bytes([0x00, 0x10]) + pw1 + pw2[:2]
    f2_7 = pw2[2:] + bytes([0x00, read_write, ucb, 0x00, 0x00])
    f2 = f2_7 + bytes([xor_chksum15(f1, f2_7)])
    return f1, f2


def read_uncompressed_frames(addr: int, size: int):
    """simos8_read_uncompressed setup: cmd 00 0A, addr BE(4), size BE(4) split hi2/lo2."""
    a = addr.to_bytes(4, "big")
    s = size.to_bytes(4, "big")
    f1 = bytes([0x00, 0x0A]) + a + s[:2]
    f2_7 = s[2:] + bytes([0x00, 0x00, 0x00, 0x00, 0x00])
    f2 = f2_7 + bytes([xor_chksum15(f1, f2_7)])
    return f1, f2


def erase_frames(addr: int, size: int):
    """erase_sector_simos8: cmd 00 04, addr BE(4), size BE(4) split hi2/lo2."""
    a = addr.to_bytes(4, "big")
    s = size.to_bytes(4, "big")
    f1 = bytes([0x00, 0x04]) + a + s[:2]
    f2_7 = s[2:] + bytes([0x00, 0x00, 0x00, 0x00, 0x00])
    f2 = f2_7 + bytes([xor_chksum15(f1, f2_7)])
    return f1, f2


# ----------------------------------------------------------------------------
# Cerberus serial transport (the CANX primitive)
# ----------------------------------------------------------------------------
class Cerberus:
    def __init__(self, port, baud=115200):
        import serial  # pip install pyserial
        self.s = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def _line(self, cmd, settle):
        self.s.reset_input_buffer()
        self.s.write((cmd + "\n").encode())
        t0 = time.time()
        buf = b""
        out = []
        while time.time() - t0 < settle:
            buf += self.s.read(self.s.in_waiting or 1)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                ln = raw.decode(errors="replace").strip()
                if ln:
                    out.append(ln)
                if ln.startswith(("DONE:", "OK:", "ERR")):
                    return out
        return out

    def ping(self):
        return any("PONG" in l for l in self._line("PING", 1.0))

    def canx(self, can_id, data: bytes, waitms=0):
        """Send one classic frame on Head 1; return [(id, bytes), ...] of RX frames."""
        cmd = "CANX:%d:%X:%s" % (BUS, can_id, data.hex())
        settle = 0.4 + waitms / 1000.0
        if waitms:
            cmd += ":%d" % waitms
        rx = []
        for ln in self._line(cmd, settle):
            if ln.startswith("RX:"):
                p = ln.split(":")
                if len(p) >= 3:
                    rx.append((int(p[1], 16), bytes.fromhex(p[2])))
        return rx

    def send(self, can_id, data: bytes):
        """Fire-and-forget single frame (waitms=0)."""
        self.canx(can_id, data, 0)


# ----------------------------------------------------------------------------
# BSL operations (Tier 1)
# ----------------------------------------------------------------------------
class TC1796BSL:
    def __init__(self, cerb: Cerberus):
        self.c = cerb

    def upload(self, bin_path):
        data = open(bin_path, "rb").read()
        print("bootloader.bin: %d bytes, %d blocks" % (len(data), (len(data) + 7) // 8))
        print("Sending BSL init (id 0x%03X)..." % ID_INIT)
        rx = self.c.canx(ID_INIT, init_frame(len(data)), waitms=500)
        if not any(cid == ID_ACK for cid, _ in rx):
            print("  no ACK on id 0x%02X. Got: %s" % (ID_ACK, rx))
            print("  -> check HWCFG strap / CAN-transmitter 3V3 wake / wiring.")
            return False
        print("  ACK 0x%02X received -- ECU in CAN BSL." % ID_ACK)
        print("Streaming %d blocks @ id 0x%02X..." % ((len(data) + 7) // 8, ID_DATA))
        for off in range(0, len(data), 8):
            self.c.send(ID_DATA, data[off:off + 8])
        print("Upload complete. RAM bootloader should be running.")
        return True

    def read_device_id(self):
        # 01 00.. on id 0x300>>2=0xC0 -> two response frames, 6 id bytes each
        rx = self.c.canx(ID_DATA, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]), waitms=200)
        out = bytearray()
        for _, d in rx:
            if d and d[0] == 0x01:
                out += d[2:8]
        return bytes(out)

    def read_addr(self, addr: int) -> bytes:
        """Read 4 bytes at addr. Returns the 4 response bytes (memory order)."""
        f1, f2 = read_frames(addr)
        self.c.send(ID_DATA, f1)
        rx = self.c.canx(ID_DATA, f2, waitms=200)
        for _, d in rx:
            if len(d) >= 4:
                return bytes(d[0:4])
        return b""

    def send_passwords(self, pw8: bytes, read_write: int):
        assert len(pw8) == 8
        f1, f2 = password_frames(pw8[0:4], pw8[4:8], read_write=read_write)
        self.c.send(ID_DATA, f1)
        rx = self.c.canx(ID_DATA, f2, waitms=300)
        ok = any(d and d[0] == BSL_SUCCESS for _, d in rx)
        print("  %s passwords: %s" % ("READ" if read_write == 0 else "WRITE",
                                       "OK (0x55)" if ok else "no/again -- %s" % rx))
        return ok

    def readmem(self, addr, size, out_path):
        """Slow but reliable region read via the read_addr loop (4 bytes/word).
        Good for the OTP/password area, identity, small CAL chunks, proving the path.
        Full multi-MB dumps want the v2 on-device streamed read (see doc)."""
        out = bytearray()
        t0 = time.time()
        for a in range(addr, addr + size, 4):
            w = self.read_addr(a)
            if len(w) != 4:
                print("  read failed at 0x%08X" % a)
                break
            out += w
            if (a - addr) % 256 == 0:
                done = a - addr + 4
                print("  0x%08X  %d/%d B  %.0f B/s" %
                      (a, done, size, done / max(0.001, time.time() - t0)))
        open(out_path, "wb").write(out[:size])
        print("wrote %d bytes -> %s" % (len(out[:size]), out_path))


# ----------------------------------------------------------------------------
# Offline self-test -- proves the framing without any hardware
# ----------------------------------------------------------------------------
def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        g = got.hex() if isinstance(got, (bytes, bytearray)) else got
        w = want.hex() if isinstance(want, (bytes, bytearray)) else want
        flag = "OK " if g == w else "FAIL"
        if g != w:
            ok = False
        print("  [%s] %-22s got=%s want=%s" % (flag, name, g, w))

    # init frame for the real 7292-byte bootloader.bin -> 912 blocks (0x0390)
    chk("init(7292)", init_frame(7292), bytes.fromhex("5555000190030003"))
    # read_byte(0xD4000C00): checksum 0xD0
    f1, f2 = read_frames(0xD4000C00)
    chk("read.f1", f1, bytes.fromhex("0008D4000C000000"))
    chk("read.f2", f2, bytes.fromhex("00000000000000D0"))
    # password frames for ABF42550 / 8513C273 -> checksum 0x1D
    f1, f2 = password_frames(bytes.fromhex("ABF42550"), bytes.fromhex("8513C273"), read_write=0)
    chk("pw.f1", f1, bytes.fromhex("0010ABF425508513"))
    chk("pw.f2", f2, bytes.fromhex("C2730000000000") + bytes([0x1D]))
    # read_uncompressed setup for (0xA0000000, 0x001FFFFF)
    f1, f2 = read_uncompressed_frames(0xA0000000, 0x001FFFFF)
    chk("rdunc.f1", f1, bytes.fromhex("000AA0000000001F"))
    # erase CAL (0xA0040000, 0x40000)
    f1, f2 = erase_frames(0xA0040000, 0x00040000)
    chk("erase.f1", f1, bytes.fromhex("0004A00400000004"))
    print("\nself-test:", "PASS" if ok else "FAIL")
    return ok


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("--selftest", "selftest"):
        sys.exit(0 if selftest() else 1)

    port = args[0]
    cmd = args[1] if len(args) > 1 else "ping"
    rest = args[2:]
    bin_path = "bootloader.bin"
    if "--bin" in rest:
        i = rest.index("--bin")
        bin_path = rest[i + 1]
        rest = rest[:i] + rest[i + 2:]

    c = Cerberus(port)
    if not c.ping():
        print("warning: no PONG from Cerberus on %s (continuing)" % port)
    bsl = TC1796BSL(c)

    if cmd == "ping":
        print("PONG" if c.ping() else "no response")
    elif cmd == "upload":
        bsl.upload(bin_path)
    elif cmd == "readid":
        print("device id:", bsl.read_device_id().hex())
    elif cmd == "read":
        addr = int(rest[0], 16)
        w = bsl.read_addr(addr)
        print("0x%08X = %s  (LE u32 0x%08X)" %
              (addr, w.hex(), int.from_bytes(w, "little")) if w else "no data")
    elif cmd == "unlock-read":
        bsl.send_passwords(bytes.fromhex(rest[0]), read_write=0x00)
    elif cmd == "unlock-write":
        bsl.send_passwords(bytes.fromhex(rest[0]), read_write=0x01)
    elif cmd == "readmem":
        bsl.readmem(int(rest[0], 16), int(rest[1], 16), rest[2] if len(rest) > 2 else "dump.bin")
    else:
        print("unknown command:", cmd)
        print(__doc__)


if __name__ == "__main__":
    main()
