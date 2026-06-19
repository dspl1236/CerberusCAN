"""
KWP1281 measuring block value decode formulas.

Each measuring block cell contains:
  - formula byte (identifies how to decode the value)
  - two raw data bytes (A and B)

The formula byte determines the calculation:
  value = f(A, B) with units

Reference: NefMoto forums, Ross-Tech documentation, community reverse engineering.
These are the same formulas VCDS uses internally.

Formula table: key = formula byte, value = (name, fn(A,B), unit)
"""

from typing import Callable, NamedTuple


class FormulaEntry(NamedTuple):
    name:    str
    fn:      Callable[[int, int], float]
    unit:    str
    fmt:     str = "{:.1f}"   # display format string


def _rpm(a, b):          return (a * 256 + b) * 0.25
def _pct(a, b):          return (a * 256 + b) * 0.01
def _deg_c(a, b):        return (a * 256 + b) * 0.1 - 273.15
def _voltage(a, b):      return (a * 256 + b) * 0.001
def _ms(a, b):           return (a * 256 + b) * 0.001
def _kpa(a, b):          return (a * 256 + b) * 0.01
def _lambda(a, b):       return (a * 256 + b) * 0.0001 + 0.5   # offset encoding
def _maf(a, b):          return (a * 256 + b) * 0.01           # g/s
def _degrees(a, b):      return (a * 256 + b) * 0.1 - 100      # ignition advance
def _raw(a, b):          return a * 256 + b
def _binary(a, b):       return a                               # status bits in a
def _temperature(a, b):  return a * 0.8 - 48                    # KWP1281 formula 0x14
def _speed(a, b):        return (a * 256 + b) * 0.01           # km/h
def _throttle(a, b):     return (a * 256 + b) * 0.01           # %
def _injection(a, b):    return (a * 256 + b) * 0.001          # ms injection time
def _factor(a, b):       return (a * 256 + b) * 0.001          # multiplicative factor


FORMULA: dict[int, FormulaEntry] = {
    # ── Engine speed ──────────────────────────────────────────────────────────
    0x08: FormulaEntry("Engine Speed",        _rpm,         "RPM",    "{:.0f}"),

    # ── Temperature ───────────────────────────────────────────────────────────
    0x12: FormulaEntry("Temperature",         _deg_c,       "°C",     "{:.1f}"),

    # ── Voltage ───────────────────────────────────────────────────────────────
    0x07: FormulaEntry("Voltage",             _voltage,     "V",      "{:.3f}"),
    0x11: FormulaEntry("Voltage (0.001V)",    _voltage,     "V",      "{:.3f}"),

    # ── Percentage ────────────────────────────────────────────────────────────
    0x04: FormulaEntry("Load / Duty Cycle",   _pct,         "%",      "{:.1f}"),
    0x10: FormulaEntry("Percentage",          _pct,         "%",      "{:.1f}"),

    # ── Lambda / AFR ──────────────────────────────────────────────────────────
    0x05: FormulaEntry("Lambda",              _lambda,      "λ",      "{:.4f}"),
    0x27: FormulaEntry("Lambda (alt)",        lambda a,b: (a*256+b)*0.00006104+0.5,
                                                            "λ",      "{:.4f}"),

    # ── MAF ───────────────────────────────────────────────────────────────────
    0x02: FormulaEntry("MAF (g/s)",           _maf,         "g/s",    "{:.2f}"),
    0x06: FormulaEntry("MAF (kg/h)",          lambda a,b: (a*256+b)*0.036, "kg/h", "{:.2f}"),

    # ── Ignition timing ───────────────────────────────────────────────────────
    0x09: FormulaEntry("Ignition Timing",     _degrees,     "° BTDC", "{:.1f}"),
    0x0A: FormulaEntry("Timing (alt)",        lambda a,b: a*0.5-64,   "°",     "{:.1f}"),

    # ── Pressure ──────────────────────────────────────────────────────────────
    0x0B: FormulaEntry("Pressure (kPa)",      _kpa,         "kPa",    "{:.1f}"),
    0x0C: FormulaEntry("Pressure (mbar)",     lambda a,b: (a*256+b)*0.01, "mbar", "{:.0f}"),

    # ── Time / injection ──────────────────────────────────────────────────────
    0x0D: FormulaEntry("Injection Time",      _injection,   "ms",     "{:.3f}"),
    0x0E: FormulaEntry("Time (ms)",           _ms,          "ms",     "{:.1f}"),

    # ── Speed ─────────────────────────────────────────────────────────────────
    0x0F: FormulaEntry("Vehicle Speed",       _speed,       "km/h",   "{:.0f}"),

    # ── Status / binary ───────────────────────────────────────────────────────
    0x03: FormulaEntry("Binary / Status",     _binary,      "",       "{}"),
    0x25: FormulaEntry("Status Bits",         _binary,      "",       "{:08b}"),

    # ── Throttle ──────────────────────────────────────────────────────────────
    0x13: FormulaEntry("Throttle Position",   _throttle,    "%",      "{:.2f}"),

    # ── Temperature (simple) ──────────────────────────────────────────────────
    0x14: FormulaEntry("Temperature (°C-48)", _temperature, "°C",     "{:.0f}"),

    # ── Adaptation factor ─────────────────────────────────────────────────────
    0x17: FormulaEntry("Adapt Factor",        _factor,      "",       "{:.3f}"),

    # ── Raw / unknown ─────────────────────────────────────────────────────────
    0x01: FormulaEntry("Raw (counts)",        _raw,         "",       "{:.0f}"),
    0xFF: FormulaEntry("Raw (alt)",           _raw,         "",       "{:.0f}"),
}


def decode_cell(formula: int, a: int, b: int) -> tuple[float, str, str]:
    """
    Decode a KWP1281 measuring block cell.

    Returns (value, unit, display_string).
    Falls back to raw decode for unknown formula bytes.
    """
    entry = FORMULA.get(formula)
    if entry is None:
        # Unknown formula — return raw word value
        raw = a * 256 + b
        return float(raw), "", f"{raw} (formula=0x{formula:02X})"

    try:
        value = float(entry.fn(a, b))
        display = entry.fmt.format(value)
        unit = entry.unit
        return value, unit, f"{display} {unit}".strip()
    except Exception:
        raw = a * 256 + b
        return float(raw), "", f"decode_err raw={raw}"
