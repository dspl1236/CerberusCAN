"""
K-line (KW1281 / KWP2000) DTC + module decode — the VAG fault-LOCATION namespace.

This is SEPARATE from the CAN/UDS path: UDS service 0x19 returns SAE J2012 3-byte
codes (Pxxxx-FT), decoded by decode_dtc() in cerberus_console.py. K-line ECUs
(KW1281 0xFC / KWP2000 0x18) report a 16-bit VAG fault-location number instead —
decoded here against the DIDB. Never cross-wire the two. (See didb/CREDITS.md.)
"""
import didb


def vag_code(code: int) -> str:
    """Format a VAG fault-location number as its 5-digit decimal code, e.g. 525 -> '00525'."""
    return f"{code:05d}"


def fault_text(code: int) -> str:
    """English description for a K-line DTC; safe fallback if the code isn't in the DIDB."""
    d = didb.dtc_description(code)
    return d if d else f"Fault {code:05d} — unknown"


def fault_line(hi: int, lo: int, status: int = None) -> str:
    """Decode one K-line fault record (hi, lo[, status]) -> 'NNNNN — description [status]'.

    0xFFFF is the KW1281 'no fault' placeholder — callers should skip it before this.
    """
    code = (hi << 8) | lo
    s = "" if status is None else f"   status=0x{status:02X}"
    return f"{vag_code(code)} — {fault_text(code)}{s}"


def module(address: int) -> str:
    """Friendly name for a K-line diagnostic ADDRESS byte (0x01 -> 'Engine Control Module 1').
    NOTE: this address byte is NOT a CAN tx/rx id — don't use it for UDS module labels."""
    return didb.module_name(address)
