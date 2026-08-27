#pragma once
// Cerberus — bus + ISO-TP configuration

#define CERBERUS_VERSION "0.9.23"

// Settle window (ms) that SNIFF/do_sniff drains before it starts counting. A head is
// re-inited (setBaudRate + enableFIFO) on entry to LISTEN-ONLY, and the stale mailbox
// entries that leaves read back as id 0 / len 0 a short moment later -- not instantly.
#define SNIFF_SETTLE_MS 5

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

// ---- Teensy pin map ----
//   Head 1  CAN1:    TX 22, RX 23   active VCI (diag bus, 500k)
//   Head 2  CAN2:    TX  1, RX  0   listen-only logger (2nd HVD230 on 6/14)
//   Head 3  Serial2: TX  8, RX  7   K-line / KWP2000 for pre-CAN VAG, via a K-line transceiver on OBD 7
//                                   (repurposed from the CAN3 30/31 spare; K-line is UART, not CAN)
//   OLED I2C0: SDA 18, SCL 19 (optional HUD)
//   TERM (reserved, planned): GPIO 2 -> TS5A3157 (10R Ron) + 110R = ~120R across OBD 6/14.
//     Run the switch on the 3V3 rail and put it on the CANL leg (CANH-[110R]-X-[switch]-CANL):
//     switch node stays ~1.5-2.5V (in range), and at V+=3V3 its VIH=0.7*V+=2.3V so the 3V3 GPIO
//     drives it directly (NO level shifter). [TS5A3157 logic thresholds are ratiometric -> a 5V
//     rail would need VIH=3.5V > 3V3.] Bench-only; default OUTPUT LOW = open = car-safe.
