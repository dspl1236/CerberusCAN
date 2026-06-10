#pragma once
// Cerberus — bus + ISO-TP configuration

// ---- Bus baud rates ----
#define BUS1_BAUD   500000   // Head 1: Powertrain/Diagnostic CAN  (OBD pins 6/14)
#define BUS2_BAUD   100000   // Head 2: Comfort/Convenience CAN     (OBD pins 3/11, FT transceiver)
// Head 3: CAN-FD (CAN3) — configured when brought up

// ---- ISO-TP ----
#define PAD_BYTE        0x00 // frame padding (some VAG modules prefer 0xAA/0xCC — change if a bus is fussy)
#define UDS_TIMEOUT_MS  1500 // per-frame response window

// ---- Teensy 4.1 FlexCAN pin map (fixed by the i.MX RT1062 mux) ----
//   Head 1  CAN1: TX 22, RX 23
//   Head 2  CAN2: TX  1, RX  0   <- LOW-SPEED FAULT-TOLERANT transceiver
//   Head 3  CAN3: TX 31, RX 30   <- CAN-FD capable
