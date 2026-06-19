"""
kwp_to_bin — reassemble a KWP2000 memory-read dump into a flat firmware image.

A KWP/UDS ReadMemoryByAddress (service 0x23) dump is a stream of addressed blocks.
Capturing those over the bus (Cerberus K-line / Head 3) gives you a `.kwp`-style file
of `[addr][len][data]` records; this maps them back into one contiguous `.bin` at their
real addresses, filling the gaps so the result loads straight into Ghidra/IDA.

Clean-room reimplementation of the block-reassembly behaviour of SafetyUggs' `kwp_stripper`
(format inferred from its output: 24-bit addr + 8-bit len + data, gap notes, size clamp).
**The exact on-disk record layout is INFERRED** — verify byte order against a real `.kwp`
before trusting an extraction (see ADDR_BYTES / ADDR_ENDIAN / LEN_BYTES knobs).

    python kwp_to_bin.py dump.kwp out.bin          # reassemble
    python kwp_to_bin.py                            # self-test
"""
import struct
import sys

# --- record layout (inferred — adjust if a real .kwp disagrees) ---
ADDR_BYTES = 3          # "Addr=0x%06X"  -> 24-bit address
ADDR_ENDIAN = "big"     # automotive addresses are normally big-endian
LEN_BYTES = 1           # "Length=0x%02X" -> 1-byte length (<=255)
FILL = 0xFF             # gap fill (0xFF = erased flash); use 0x00 if you prefer


def parse_blocks(data: bytes):
    """Yield (addr, payload) records from a .kwp blob. Stops cleanly on a short tail."""
    off, n = 0, len(data)
    hdr = ADDR_BYTES + LEN_BYTES
    while off + hdr <= n:
        addr = int.from_bytes(data[off:off + ADDR_BYTES], ADDR_ENDIAN)
        length = int.from_bytes(data[off + ADDR_BYTES:off + hdr], "big")
        body = data[off + hdr:off + hdr + length]
        if len(body) < length:                      # truncated final record
            break
        yield addr, body
        off += hdr + length


def reassemble(data: bytes, fill: int = FILL, base: int = None, size: int = None):
    """Map all blocks into a flat image. Returns (image_bytes, base_addr, stats)."""
    blocks = list(parse_blocks(data))
    if not blocks:
        return b"", 0, {"blocks": 0, "gaps": [], "bytes": 0}
    lo = base if base is not None else min(a for a, _ in blocks)
    hi = max(a + len(b) for a, b in blocks)
    if size is not None:
        hi = lo + size
    img = bytearray([fill]) * (hi - lo)
    written, gaps, prev_end = 0, [], lo
    for addr, body in sorted(blocks):
        if addr < lo or addr + len(body) > hi:      # "exceeds output file size, skipping"
            continue
        if addr > prev_end:
            gaps.append((prev_end, addr))
        img[addr - lo:addr - lo + len(body)] = body
        written += len(body)
        prev_end = max(prev_end, addr + len(body))
    return bytes(img), lo, {"blocks": len(blocks), "gaps": gaps, "bytes": written}


def _selftest():
    # synthetic .kwp: three 24-bit-addr / 1-byte-len blocks with a gap between #2 and #3
    def rec(addr, body):
        return addr.to_bytes(3, "big") + len(body).to_bytes(1, "big") + body
    blob = rec(0x000000, b"\xDE\xAD") + rec(0x000002, b"\xBE\xEF") + rec(0x000010, b"\x42\x42")
    img, base, st = reassemble(blob, fill=0xFF)
    print(f"base=0x{base:06X} len={len(img)} blocks={st['blocks']} "
          f"wrote={st['bytes']} gaps={[(hex(a),hex(b)) for a,b in st['gaps']]}")
    print(img.hex())
    assert base == 0x0000 and len(img) == 0x12
    assert img[0:4] == b"\xDE\xAD\xBE\xEF"          # contiguous blocks merged
    assert img[4:0x10] == b"\xFF" * 12              # gap filled
    assert img[0x10:0x12] == b"\x42\x42"
    assert st["gaps"] == [(0x04, 0x10)]
    print("kwp_to_bin self-test OK")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        with open(sys.argv[1], "rb") as fh:
            blob = fh.read()
        img, base, st = reassemble(blob)
        with open(sys.argv[2], "wb") as fh:
            fh.write(img)
        gaps = ", ".join(f"0x{a:06X}-0x{b:06X}" for a, b in st["gaps"]) or "none"
        print(f"{st['blocks']} blocks, {st['bytes']} bytes -> {sys.argv[2]} "
              f"(base 0x{base:06X}, {len(img)} bytes, gaps: {gaps})")
    else:
        _selftest()
