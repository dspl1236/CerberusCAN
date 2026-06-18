#pragma once
// Cerberus — bus + ISO-TP configuration

#define CERBERUS_VERSION "0.9.1-emu"

// ---- Bus baud rates ----
#define BUS1_BAUD   500000   // Head 1: active VCI on the Diagnostic CAN (OBD 6/14) — HS, SN65HVD230
#define BUS2_BAUD   500000   // Head 2: 2nd HVD230 tapped on the SAME diag bus (OBD 6/14),
                             //         held LISTEN-ONLY = the always-on background logger (MON).
// Two SN65HVD230s share OBD 6/14: Head 1 talks (active), Head 2 only listens (passive log).
// IMPORTANT: remove the 120R termination resistor from BOTH HVD230 breakouts — the car already
// has ~60R on the bus, so the two taps must add NO termination (high-impedance stubs only).
// Head 3: CAN-FD (CAN3, pins 30/31) needs an FD-rated transceiver — spare, unused for now.

// ---- ISO-TP ----
#define PAD_BYTE        0x00 // frame padding (some VAG modules prefer 0xAA/0xCC — change if fussy)
#define UDS_TIMEOUT_MS  6000 // per-frame response / flow-control window (VAG gateways can take ~5 s on CP DIDs like 0x00BE — keep margin)

// ---- Teensy 4.1 FlexCAN pin map (fixed by the i.MX RT1062 mux) ----
//   Head 1  CAN1: TX 22, RX 23
//   Head 2  CAN2: TX  1, RX  0
//   Head 3  CAN3: TX 31, RX 30   <- CAN-FD capable
