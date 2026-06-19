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


def ftb_text(ftb: int) -> str:
    """Meaning of the failure-type byte (e.g. 0x11 -> 'short to ground'). '' if unknown."""
    _, f = _load()
    return f.get(f"{ftb:02X}", "")


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
