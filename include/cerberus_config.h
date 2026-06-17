#pragma once
// Cerberus — bus + ISO-TP configuration

#define CERBERUS_VERSION "0.5.0-canx"

// ---- Bus baud rates ----
#define BUS1_BAUD   500000   // Head 1: Powertrain/Diagnostic CAN (OBD 6/14) — HS, SN65HVD230
#define BUS2_BAUD   100000   // Head 2: Comfort/Convenience CAN   (OBD 3/11) — HS, SN65HVD230
// Both heads use a 3.3 V SN65HVD230. Head 2 assumes the comfort bus is HIGH-SPEED at
// 100k; if it sniffs nothing the bus is LOW-SPEED FAULT-TOLERANT (ISO 11898-3) — the
// HVD230 can't read it and that head needs an FT part (TJA1055T/3). Confirm via idle
// volts on OBD 3/11 (both ~2.5 V = HS; split ~0 V / ~5 V = FT).
// Head 3: CAN-FD (CAN3, pins 30/31) needs an FD-rated transceiver, NOT the SN65HVD230.

// ---- ISO-TP ----
#define PAD_BYTE        0x00 // frame padding (some VAG modules prefer 0xAA/0xCC — change if fussy)
#define UDS_TIMEOUT_MS  6000 // per-frame response / flow-control window (VAG gateways can take ~5 s on CP DIDs like 0x00BE — keep margin)

// ---- Teensy 4.1 FlexCAN pin map (fixed by the i.MX RT1062 mux) ----
//   Head 1  CAN1: TX 22, RX 23
//   Head 2  CAN2: TX  1, RX  0
//   Head 3  CAN3: TX 31, RX 30   <- CAN-FD capable
