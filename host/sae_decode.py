"""
SAE J2012 / ISO 15031-6 generic OBD-II DTC text + ISO 14229 failure-type-byte meanings.

For the CAN/UDS path (service 0x19): a 3-byte DTC decodes to an SAE code (Pxxxx/Cxxxx/
Bxxxx/Uxxxx) + a failure-type byte (the "-XX"). This adds the English text. This is the
SAE namespace — SEPARATE from the VAG fault-location DIDB used for K-line (kline_decode.py).

Data lives in saedb/{pcodes.json, ftb.json} (generated + vetted). Missing data degrades
gracefully to just the code, so the Console works with or without it.
"""
import json
import os

_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saedb")
_pcodes = None
_ftb = None


def _load():
    global _pcodes, _ftb
    if _pcodes is None:
        try:
            with open(os.path.join(_DIR, "pcodes.json"), encoding="utf-8") as fh:
                _pcodes = json.load(fh)
        except Exception:
            _pcodes = {}
        try:
            with open(os.path.join(_DIR, "ftb.json"), encoding="utf-8") as fh:
                _ftb = json.load(fh)
        except Exception:
            _ftb = {}
    return _pcodes, _ftb


def pcode_text(code: str) -> str:
    """Standard generic description for an SAE code (e.g. 'P0420' -> 'Catalyst ...'). '' if unknown."""
    p, _ = _load()
    return p.get(code.upper(), "")


# high-nibble fallback for failure-type bytes that didn't reach consensus
_FTB_GROUPS = {
    0x0: "general failure", 0x1: "electrical/circuit", 0x2: "signal range/performance",
    0x3: "signal timing/frequency", 0x4: "ECU internal/memory", 0x5: "configuration/calibration",
    0x6: "signal monitoring/plausibility", 0x7: "actuator/mechanical", 0x8: "bus/network",
    0x9: "component/system operation", 0xA: "calibration/parameter memory",
}


def ftb_text(ftb: int) -> str:
    """Meaning of the failure-type byte (e.g. 0x11 -> 'Circuit short to ground'). Falls back to the
    high-nibble group meaning (with a '?') for bytes without a consensus entry; '' if neither."""
    _, f = _load()
    exact = f.get(f"{ftb:02X}", "")
    if exact:
        return exact
    grp = _FTB_GROUPS.get((ftb >> 4) & 0xF)
    return f"{grp}?" if grp else ""


def describe(code: str, ftb: int) -> str:
    """One-line annotation for a UDS DTC: '<SAE text> [<FTB text>]' (each part omitted if unknown)."""
    t = pcode_text(code)
    ft = ftb_text(ftb)
    if t and ft:
        return f"{t}  [{ft}]"
    if t:
        return t
    if ft:
        return f"[{ft}]"
    return ""
