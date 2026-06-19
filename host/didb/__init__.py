"""
kwpbridge/didb — VAG Diagnostic Information Database (DIDB)

Extracted from didb_Base + didb_Base-en_US (HSQL 1.8, March 2023).
Provides:
  - dtc_description(code)      — English DTC description for VAG decimal code
  - module_name(address)       — Module long name for a KWP address byte
  - module_symbol(address)     — VAG module symbol (e.g. "MOT_01")
  - MODULE_MAP                 — dict[int, dict] for all 89 modules

Coverage: 4,817 DTC descriptions (codes 0–65535, dense in 0–4818).
          89 ECU module addresses (0x01–0x99).

These are the OFFICIAL VAG descriptions from the dealer diagnostic system,
used for display in KWPBridge fault code readout and module identification.
"""

import json
import os

_DIR = os.path.dirname(__file__)

# ── Lazy-load data ────────────────────────────────────────────────────────────

_dtc: dict[int, str] | None = None
_modules: list[dict] | None = None
_module_map: dict[int, dict] | None = None


def _load_dtc():
    global _dtc
    if _dtc is None:
        with open(os.path.join(_DIR, 'dtc_descriptions.json')) as f:
            raw = json.load(f)
        _dtc = {int(k): v for k, v in raw.items()}
    return _dtc


def _load_modules():
    global _modules, _module_map
    if _modules is None:
        with open(os.path.join(_DIR, 'modules.json')) as f:
            _modules = json.load(f)
        _module_map = {m['address']: m for m in _modules}
    return _modules, _module_map


# ── Public API ────────────────────────────────────────────────────────────────

def dtc_description(code: int) -> str:
    """
    Return the English DTC description for a VAG decimal fault code.

    Returns empty string if the code is not in the DIDB.

    Example:
        dtc_description(525)  -> "Oxygen sensor"
        dtc_description(533)  -> "Idle speed control"
        dtc_description(0)    -> "End of output"
    """
    return _load_dtc().get(code, "")


def module_name(address: int) -> str:
    """
    Return the long module name for a KWP/OBD module address.

    Example:
        module_name(0x01)  -> "Engine Control Module 1"
        module_name(0x02)  -> "Transmission Control Module"
        module_name(0x15)  -> "Airbag"
    """
    _, mm = _load_modules()
    m = mm.get(address)
    return m['longname'] if m else f"Module 0x{address:02X}"


def module_symbol(address: int) -> str:
    """
    Return the VAG module symbol for an address (e.g. "MOT_01").

    Example:
        module_symbol(0x01)  -> "MOT_01"
        module_symbol(0x02)  -> "GET_02"
    """
    _, mm = _load_modules()
    m = mm.get(address)
    return m['symbol'] if m else ""


def module_info(address: int) -> dict | None:
    """
    Return the full module dict for an address, or None.

    Keys: address, address_hex, longname, symbol, shortname
    """
    _, mm = _load_modules()
    return mm.get(address)


def all_modules() -> list[dict]:
    """Return the full list of all 89 DIDB module records, sorted by address."""
    mods, _ = _load_modules()
    return mods


@property
def MODULE_MAP() -> dict[int, dict]:
    """dict[int, dict] — all modules keyed by address integer."""
    _, mm = _load_modules()
    return mm
