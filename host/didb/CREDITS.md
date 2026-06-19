# DIDB — VAG Diagnostic Information Database

`dtc_descriptions.json` (4,817 fault descriptions) and `modules.json` (89 ECU
addresses) are the VAG Diagnostic Information Database, extracted from the dealer
`didb_Base` + `didb_Base-en_US` (HSQL 1.8, March 2023). Ported verbatim from
[dspl1236/KWPBridge](https://github.com/dspl1236/KWPBridge) (`kwpbridge/didb/`).

**What it decodes:** the legacy **VAG decimal fault-location** numbering used by
**K-line** ECUs (KW1281 `0xFC` and KWP2000 `0x18`/readDTCByStatus) — the 16-bit
`(hi<<8)|lo` fault code is the lookup key. The module map is keyed by the
**diagnostic address byte** (e.g. `0x01` engine, `0x19` gateway).

**Not** for the CAN/UDS path: UDS service `0x19` returns SAE J2012 3-byte codes
(`Pxxxx-FT`), a different namespace — those are decoded separately in the Console
(`decode_dtc()`), never via this table.

Provenance: dealer-derived VAG reference data. Loader is dependency-free stdlib Python.
