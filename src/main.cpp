/*
 * Cerberus — Teensy 4.1 tri-CAN OBD interface for VAG
 *
 *   Head 1: Diagnostic CAN  (CAN1, 500 kbps, OBD 6/14)     -- active VCI (UDS read/write/CP)
 *   Head 2: 2nd tap, SAME bus (CAN2, 500 kbps, OBD 6/14)   -- LISTEN-ONLY always-on logger (MON)
 *   Head 3: CAN-FD spare    (CAN3, pins 30/31)             -- stub (needs FD xcvr)
 *
 * A USB-serial bridge. On either classic bus it runs a full ISO-TP/UDS exchange
 * (read AND write — single- and multi-frame, with flow control), passively sniffs,
 * and now reports WHERE an exchange fails so gateway-routing problems are diagnosable.
 *
 * Line protocol (115200, ASCII) — one command per line:
 *
 *   <TXID>:<RXID>:<HEX>             UDS on Head 1 (shorthand)      710:77A:2200BE
 *   UDS:<bus>:<TXID>:<RXID>:<HEX>   UDS on bus 1|2 (explicit)      UDS:1:710:77A:2E00BE..
 *   RAW:<bus>:<ID>:<HEX>           send ONE raw frame (<=8B), no ISO-TP   RAW:1:710:023E00
 *   SCAN:<bus>[:lo:hi[:winms]]     TesterPresent sweep (winms reply window, default 120)
 *   SNIFF:<bus>:<ms>[:idlo:idhi]   passive LISTEN-ONLY dump (no ACK; ms=0=until byte; optional ID range)
 *   MON:on[:idlo:idhi] | MON:off   always-on Head-2 background logger -> M2:<ms>:<id>:<hex> lines
 *                                  (runs concurrently with active Head-1 UDS/CP — capture while you drive)
 *   STATS:<bus>                    CAN controller health: error counters / fault state / RX overrun
 *   TP:<bus>:<TXID>:<ms>          background TesterPresent keep-alive (non-blocking) | TP:STOP to end
 *   INFO                           firmware + bus config
 *   PING                           -> PONG
 *   SLCAN                          enter Lawicel SLCAN mode on Head 1 (SavvyCAN / slcand / python-can);
 *                                  also auto-entered on the first SLCAN cmd (O/C/L/S<n>/t/T). Reset to exit.
 *
 * Replies:  OK:<resphex>
 *           ERR:tx-fail | ERR:no-flow-control | ERR:no-response | ERR:partial:<got>/<total>
 *           (sniff) RX:<ms>:<id>:<data> ... DONE:<n>
 *
 * Diagnostic ERR codes (the point of v0.3.0):
 *   tx-fail          our frame never made it onto the wire (no xcvr/bus ACK -> bus down/idle)
 *   no-flow-control  module RX'd our First Frame but never sent CTS (busy / routing dropped it)
 *   no-response      NOTHING came back on RXID  (wrong response ID / module not routed / silent)
 *   partial:g/t      we got a First Frame + g of t bytes, then it went quiet (routed-bus stall)
 *
 * Writes need no special command: a request payload >7 bytes auto First-Frame/CF frames.
 *
 * GPLv3. github.com/dspl1236/CerberusCAN
 */
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "cerberus_config.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Head1;   // diagnostic, 500k  (OBD 6/14)
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Head2;   // comfort,    100k  (OBD 3/11)
// FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> Head3;  // CAN-FD (pins 30/31), classic-CAN C7 never uses it.

// ---------------- hex helpers ----------------
static uint8_t nib(char c){
  if (c>='0'&&c<='9') return c-'0';
  if (c>='a'&&c<='f') return c-'a'+10;
  if (c>='A'&&c<='F') return c-'A'+10;
  return 0xFF;
}
static int hexBytes(const String& s, uint8_t* out, int maxlen){
  int n=s.length(); if (n&1) return -1; int b=0;
  for (int i=0;i<n && b<maxlen;i+=2){ uint8_t hi=nib(s[i]),lo=nib(s[i+1]); if(hi>15||lo>15)return -1; out[b++]=(hi<<4)|lo; }
  return (b*2==n)?b:-1;                              // -1 if it didn't all fit
}
static void printHex(const uint8_t* d,int n){ for(int i=0;i<n;i++){ if(d[i]<16)Serial.print('0'); Serial.print(d[i],HEX);} }

// ---------------- Head 2 always-on background logger (ring-buffered) ----------------
// Head 2 is a SECOND SN65HVD230 in parallel with Head 1 on OBD 6/14, held LISTEN_ONLY.
// Capture is decoupled from USB by a big ring buffer in OCRAM: mon_capture() drains the
// FlexCAN FIFO into the ring (fast, never blocks) and mon_flush() drains the ring to USB
// only when there's TX room (never blocks). So a USB stall — or a long blocking Head-1
// transaction — no longer overflows the 256-deep FIFO; the ring (16384 frames / 256 KB)
// absorbs the burst and drains afterward. Frames stream as M2:<ms>:<id>:<hex>[:OVR].
// If the ring itself saturates, the newest frame is dropped and counted (MON:stat / MON:off).
struct MonFrame { uint32_t t; uint16_t id; uint8_t len; uint8_t ovr; uint8_t buf[8]; };  // 16 B
#define MON_RING 16384                                // 16384 * 16 B = 256 KB (OCRAM)
DMAMEM static MonFrame mon_ring[MON_RING];
static bool     mon_on   = false;
static uint32_t mon_idlo = 0x000, mon_idhi = 0x7FF;
static uint32_t mon_t0   = 0;
static uint32_t mon_head = 0, mon_tail = 0;           // head = write idx, tail = read idx
static uint32_t mon_dropped = 0, mon_peak = 0, mon_total = 0;
static inline bool     mon_empty(){ return mon_head==mon_tail; }
static inline bool     mon_full() { return ((mon_head+1)%MON_RING)==mon_tail; }
static inline uint32_t mon_depth(){ return (mon_head + MON_RING - mon_tail) % MON_RING; }

static void mon_capture(){                            // FIFO -> ring (fast, never blocks)
  if (!mon_on) return;
  CAN_message_t r;
  while (Head2.read(r)){
    if (r.id < mon_idlo || r.id > mon_idhi) continue;
    if (mon_full()){ mon_dropped++; continue; }       // ring saturated -> drop newest + count
    MonFrame &f = mon_ring[mon_head];
    f.t = millis()-mon_t0; f.id = (uint16_t)r.id; f.len = r.len; f.ovr = r.flags.overrun;
    for (int i=0;i<r.len && i<8;i++) f.buf[i]=r.buf[i];
    mon_head = (mon_head+1)%MON_RING; mon_total++;
  }
  uint32_t d=mon_depth(); if (d>mon_peak) mon_peak=d;
}

// ---------------- optional SSD1306 OLED status HUD ----------------
// Auto-detected on I2C0 (pin 18 SDA / 19 SCL), probing 0x3C then 0x3D (note: many boards
// silk-print 0x78, which is just the 8-bit form of 0x3C). No panel -> the probe NAKs and every
// OLED line is skipped, so the firmware runs identically with or without it. Live MON shows
// total / fps / dropped / peak. Default geometry 128x64 (the common 0.96" yellow/blue panel:
// top 16 px yellow = a title band, the rest blue); comment OLED_128x64 for a 0.91/0.92" 128x32.
#define OLED_128x64
static uint8_t oled_addr = 0;
SSD1306AsciiWire oled;
static bool     oled_present = false;
static uint32_t oled_last = 0, oled_ltot = 0, oled_lms = 0;

static bool oled_probe(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }

static void oled_init(){
  Wire.begin();
  Wire.setClock(400000);
  if      (oled_probe(0x3C)) oled_addr = 0x3C;
  else if (oled_probe(0x3D)) oled_addr = 0x3D;
  else return;                                         // no panel -> stay disabled
  oled_present = true;
#ifdef OLED_128x64
  oled.begin(&Adafruit128x64, oled_addr);
#else
  oled.begin(&Adafruit128x32, oled_addr);
#endif
  oled.setFont(System5x7);
  oled.clear();
}

static void oled_update(){
  if (!oled_present) return;
  uint32_t now = millis();
  if (now - oled_last < 250) return;                  // ~4 Hz, never per-loop (I2C is blocking)
  oled_last = now;
  uint32_t fps = 0;
  if (oled_lms && now > oled_lms)
    fps = (uint32_t)((uint64_t)(mon_total - oled_ltot) * 1000 / (now - oled_lms));
  oled_ltot = mon_total; oled_lms = now;

  // row 0 = yellow title band on the two-colour panels; rows 2+ = blue (live data)
  oled.setCursor(0, 0); oled.print("CERBERUS "); oled.print(CERBERUS_VERSION); oled.clearToEOL();
  if (mon_on){
    oled.setCursor(0, 2); oled.print("MON  "); oled.print(mon_total);
                          oled.print("  "); oled.print(fps); oled.print("/s"); oled.clearToEOL();
    oled.setCursor(0, 3); oled.print("drop "); oled.print(mon_dropped); oled.clearToEOL();
    oled.setCursor(0, 4); oled.print("peak "); oled.print(mon_peak); oled.clearToEOL();
  } else {
    oled.setCursor(0, 2); oled.print("idle"); oled.clearToEOL();
    oled.setCursor(0, 3); oled.clearToEOL();
    oled.setCursor(0, 4); oled.clearToEOL();
  }
  oled.setCursor(0, 6); oled.print("H1 VCI   H2 LOG"); oled.clearToEOL();
}

static void mon_flush(uint32_t maxframes){            // ring -> USB (only when TX has room)
  uint32_t sent=0;
  while (!mon_empty() && sent<maxframes){
    if ((uint32_t)Serial.availableForWrite() < 40) break;  // would block -> stop; ring holds it
    MonFrame &f = mon_ring[mon_tail];
    Serial.print("M2:"); Serial.print(f.t); Serial.print(':');
    Serial.print(f.id, HEX); Serial.print(':');
    printHex(f.buf, f.len);
    if (f.ovr) Serial.print(":OVR");
    Serial.println();
    mon_tail = (mon_tail+1)%MON_RING;
    sent++;
  }
}

static void pump_mon(){
  if (!mon_on) return;
  mon_capture();        // always drain the FIFO into the ring first (the anti-drop guarantee)
  mon_flush(32);        // then opportunistically push some frames out to USB
}

// ---------------- ISO-TP ----------------
static inline void stmin_delay(uint8_t st){
  if (st==0) return;
  if (st<=0x7F) delay(st);                                       // 1..127 ms
  else if (st>=0xF1 && st<=0xF9) delayMicroseconds((st-0xF0)*100); // 100..900 us
  else delay(1);                                                 // reserved -> be safe
}

// Wait for a Flow Control frame on rx. 0 = CTS (fills bs/stmin), 2 = abort/timeout.
template <typename BUS>
static int wait_fc(BUS& bus, uint32_t rx, uint8_t& bs, uint8_t& stmin){
  uint32_t dl = millis()+UDS_TIMEOUT_MS;
  while ((int32_t)(dl-millis())>0){
    pump_mon();                                     // keep the Head-2 logger draining
    CAN_message_t r;
    if (!bus.read(r)) continue;
    if (r.id != rx) continue;
    if ((r.buf[0]>>4) != 3) continue;               // not a flow-control frame
    uint8_t fs = r.buf[0]&0x0F;
    if (fs==0){ bs=r.buf[1]; stmin=r.buf[2]; return 0; }  // CTS
    if (fs==1){ dl=millis()+UDS_TIMEOUT_MS; continue; }   // WAIT -> keep waiting
    return 2;                                              // OVFLW / abort
  }
  return 2;                                                // timeout
}

// Send a UDS request of any length as ISO-TP.  0=ok, 1=tx-fail, 2=no-flow-control.
template <typename BUS>
static int isotp_send(BUS& bus, uint32_t tx, uint32_t rx, const uint8_t* req, uint16_t len){
  CAN_message_t m; m.id=tx; m.flags.extended=0; m.len=8;
  for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;

  if (len<=7){                                      // Single Frame
    m.buf[0]=len & 0x0F;
    for (uint16_t i=0;i<len;i++) m.buf[1+i]=req[i];
    return bus.write(m) ? 0 : 1;
  }

  m.buf[0]=0x10 | ((len>>8)&0x0F);                  // First Frame
  m.buf[1]=len & 0xFF;
  for (int i=0;i<6;i++) m.buf[2+i]=req[i];
  if (!bus.write(m)) return 1;
  uint16_t sent=6;

  uint8_t bs=0, stmin=0;
  if (wait_fc(bus, rx, bs, stmin)!=0) return 2;     // first FC must be CTS

  uint8_t sn=1, inblock=0;
  while (sent<len){                                 // Consecutive Frames
    for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;
    m.buf[0]=0x20 | (sn & 0x0F);
    for (int i=0;i<7 && sent<len;i++) m.buf[1+i]=req[sent++];
    if (!bus.write(m)) return 1;
    sn=(sn+1)&0x0F; inblock++;
    if (sent<len){
      stmin_delay(stmin);
      if (bs!=0 && inblock>=bs){                     // block full -> next FC
        inblock=0;
        if (wait_fc(bus, rx, bs, stmin)!=0) return 2;
      }
    }
  }
  return 0;
}

// Receive a UDS response (ISO-TP). >=0 length; -1 nothing seen; -2 partial (got<total).
// sawAny is set true if ANY frame from rx arrived; got/total report partial progress.
template <typename BUS>
static int isotp_recv(BUS& bus, uint32_t tx, uint32_t rx, uint8_t* resp, uint16_t maxresp,
                      bool& sawAny, uint16_t& got, uint16_t& total){
  uint32_t dl = millis()+UDS_TIMEOUT_MS;
  got=0; total=0; sawAny=false; bool multi=false;
  while ((int32_t)(dl-millis())>0){
    pump_mon();                                     // keep the Head-2 logger draining
    CAN_message_t r;
    if (!bus.read(r)) continue;
    if (r.id != rx) continue;
    sawAny=true;
    uint8_t pci = r.buf[0]>>4;

    if (pci==0){                                    // Single Frame
      uint8_t len=r.buf[0]&0x0F; if (len>maxresp) len=maxresp;
      for (int i=0;i<len;i++) resp[i]=r.buf[1+i];
      if (len==3 && resp[0]==0x7F && resp[2]==0x78){ dl=millis()+UDS_TIMEOUT_MS; continue; } // pending
      return len;
    }
    else if (pci==1){                               // First Frame
      total=((r.buf[0]&0x0F)<<8)|r.buf[1]; multi=true; got=0;
      for (int i=0;i<6 && got<total && got<maxresp;i++) resp[got++]=r.buf[2+i];
      CAN_message_t fc; fc.id=tx; fc.flags.extended=0; fc.len=8;
      for (int i=0;i<8;i++) fc.buf[i]=PAD_BYTE;
      fc.buf[0]=0x30; fc.buf[1]=0x00; fc.buf[2]=0x00;  // CTS, BS=0, STmin=0
      bus.write(fc);
      dl=millis()+UDS_TIMEOUT_MS;
    }
    else if (pci==2){                               // Consecutive Frame
      for (int i=0;i<7 && got<total && got<maxresp;i++) resp[got++]=r.buf[1+i];
      dl=millis()+UDS_TIMEOUT_MS;
      if (got>=total) return got;
    }
    // pci==3 (stray flow control): ignore
  }
  return (multi && got)? -2 : -1;                   // -2 = partial, -1 = nothing usable
}

template <typename BUS>
static void do_uds(BUS& bus, uint32_t tx, uint32_t rx, const uint8_t* req, uint16_t reqlen){
  int s = isotp_send(bus, tx, rx, req, reqlen);
  if (s==1){ Serial.println("ERR:tx-fail (frame never ACKed on the wire — bus down / wrong head)"); return; }
  if (s==2){ Serial.println("ERR:no-flow-control (module never sent CTS to our First Frame)"); return; }

  static uint8_t resp[4096];
  bool sawAny=false; uint16_t got=0, total=0;
  int n = isotp_recv(bus, tx, rx, resp, sizeof(resp), sawAny, got, total);
  if (n==-1){ Serial.println(sawAny ? "ERR:no-response (frames on RXID but not a usable reply)"
                                    : "ERR:no-response (NOTHING on RXID — wrong id / not routed / module silent)"); return; }
  if (n==-2){ Serial.print("ERR:partial:"); Serial.print(got); Serial.print('/'); Serial.print(total);
              Serial.println(" (First Frame seen, then the routed bus went quiet)"); return; }
  Serial.print("OK:"); printHex(resp,n); Serial.println();
}

// Send ONE raw CAN frame (no ISO-TP). For poking the gateway / debugging routing.
template <typename BUS>
static void do_raw(BUS& bus, uint32_t id, const uint8_t* data, int n){
  CAN_message_t m; m.id=id; m.flags.extended=0; m.len=8;
  for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;
  for (int i=0;i<n && i<8;i++) m.buf[i]=data[i];
  Serial.println(bus.write(m) ? "OK:sent" : "ERR:tx-fail");
}

// Send ONE classic CAN frame in NORMAL (TX) mode, then capture RX frames for waitms ms.
// The missing primitive: SNIFF is listen-only (can't TX) and RAW is send-only (can't RX).
// CANX = send-then-listen, which lets the host drive low-level request/response protocols
// like the TC1796 CAN bootstrap loader (BSL) entirely from a script — no per-tweak reflash.
// waitms==0 => fire-and-forget (fast streaming, e.g. BSL block upload).
template <typename BUS>
static void do_canx(BUS& bus, uint32_t id, const uint8_t* data, int n, uint32_t waitms){
  CAN_message_t m; m.id=id; m.flags.extended=0; m.len=(n>8)?8:n;
  for (int i=0;i<m.len;i++) m.buf[i]=data[i];
  if (!bus.write(m)){ Serial.println("ERR:tx-fail"); return; }
  if (waitms==0){ Serial.println("OK:sent"); return; }
  uint32_t start=millis(), cnt=0;
  while ((int32_t)((start+waitms)-millis())>0){
    CAN_message_t r;
    if (bus.read(r)){
      Serial.print("RX:"); Serial.print(r.id, HEX); Serial.print(':');
      printHex(r.buf, r.len); Serial.println();
      cnt++;
    }
  }
  Serial.print("DONE:"); Serial.println(cnt);
}

// Passive monitor. ms=0 -> run until any serial byte arrives. Only emits frames whose
// id is in [idlo, idhi] (software accept-range; default = full 0x000..0x7FF).
// The SNIFF handler puts the controller in hardware LISTEN-ONLY (LOM) first, so this is
// truly passive — it never ACKs and can't disturb a tester (e.g. ODIS) on the same bus.
template <typename BUS>
static void do_sniff(BUS& bus, uint32_t ms, uint32_t idlo, uint32_t idhi){
  uint32_t start=millis(), count=0, overrun=0;
  for (;;){
    if (ms!=0 && (int32_t)((start+ms)-millis())<=0) break;
    if (ms==0 && Serial.available()){ while(Serial.available()) Serial.read(); break; }
    CAN_message_t r;
    if (bus.read(r)){
      if (r.id < idlo || r.id > idhi) continue;       // ID accept-range filter
      if (r.flags.overrun) overrun++;                 // RX FIFO overran before this frame
      Serial.print("RX:"); Serial.print(millis()-start); Serial.print(':');
      Serial.print(r.id, HEX); Serial.print(':');
      printHex(r.buf, r.len); Serial.println();
      count++;
    }
  }
  Serial.print("DONE:"); Serial.print(count);
  if (overrun){ Serial.print(" overrun:"); Serial.print(overrun); }  // dropped-frame hint
  Serial.println();
}

// ---- background TesterPresent (NON-BLOCKING; serviced every loop) ----
// Holds a routed diagnostic channel open WITHOUT freezing the command interface, so a
// long flash (erase -> download -> many TransferData chunks) keeps its session alive.
static int      tp_bus  = 0;          // 0 = off, else 1|2
static uint32_t tp_tx   = 0;
static uint32_t tp_ms   = 0;
static uint32_t tp_last = 0;
static void tp_service(){
  if (!tp_bus) return;
  if (millis() - tp_last < tp_ms) return;
  CAN_message_t m; m.id = tp_tx; m.flags.extended = 0; m.len = 8;
  for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;
  m.buf[0]=0x02; m.buf[1]=0x3E; m.buf[2]=0x80;       // TesterPresent, suppress positive
  if (tp_bus==1) Head1.write(m); else if (tp_bus==2) Head2.write(m);
  tp_last = millis();
}

// Active discovery: sweep VAG 11-bit UDS request IDs, send TesterPresent, report responders.
// winms reply window now configurable — gateway-routed modules need ~100 ms, not 40.
template <typename BUS>
static void do_scan(BUS& bus, uint32_t lo, uint32_t hi, uint32_t winms){
  uint32_t found = 0;
  for (uint32_t tx = lo; tx <= hi; tx++){
    CAN_message_t m; m.id = tx; m.flags.extended = 0; m.len = 8;
    for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;
    m.buf[0]=0x02; m.buf[1]=0x3E; m.buf[2]=0x00;       // single-frame TesterPresent
    bus.write(m);
    uint32_t dl = millis() + winms;                     // reply window (routed modules are slow)
    while ((int32_t)(dl-millis())>0){
      CAN_message_t r;
      if (bus.read(r)){
        if (r.id == tx) continue;                       // ignore self-reception echo
        Serial.print("FOUND:"); Serial.print(tx, HEX); Serial.print(':');
        Serial.print(r.id, HEX); Serial.print(':');
        printHex(r.buf, r.len); Serial.println();
        found++;
        break;
      }
    }
  }
  Serial.print("DONE:"); Serial.println(found);
}

// ---------------- SLCAN (Lawicel ASCII) ----------------
// A standard serial-CAN protocol so the board drops into the wider tool ecosystem:
// SavvyCAN (direct), python-can (slcan), and slcand -> SocketCAN -> Wireshark/CANdevStudio.
// Single channel on Head 1 (the 500k diagnostic bus). Replies: '\r' ok, '\a' (BEL) error.
// Frames stream out as t<iii><l><dd..> (standard) / T<iiiiiiii><l><dd..> (extended).
bool slcan_mode = false;
static bool slcan_open = false;
static uint32_t slcan_baud = BUS1_BAUD;

static uint32_t slcan_bitrate(char c){
  switch (c){ case '0':return 10000;  case '1':return 20000;  case '2':return 50000;
              case '3':return 100000; case '4':return 125000; case '5':return 250000;
              case '6':return 500000; case '7':return 800000; case '8':return 1000000; }
  return 0;
}
static void slcan_emit(const CAN_message_t& r){
  if (r.flags.extended){ Serial.print('T'); for (int sh=28; sh>=0; sh-=4) Serial.print((r.id>>sh)&0xF, HEX); }
  else { Serial.print('t'); Serial.print((r.id>>8)&0xF,HEX); Serial.print((r.id>>4)&0xF,HEX); Serial.print(r.id&0xF,HEX); }
  Serial.print(r.len & 0xF, HEX);
  for (int i=0;i<r.len;i++){ if (r.buf[i]<16) Serial.print('0'); Serial.print(r.buf[i],HEX); }
  Serial.print('\r');
}
static bool is_slcan_cmd(const String& s){      // recognise an SLCAN line to auto-enter the mode
  if (!s.length()) return false;
  char c=s[0];
  if (c=='t'||c=='T'||c=='r'||c=='R') return true;
  if (s.length()==1 && (c=='O'||c=='C'||c=='L'||c=='V'||c=='v'||c=='F'||c=='N')) return true;
  if (c=='S' && s.length()==2 && s[1]>='0' && s[1]<='8') return true;   // S<bitrate>
  return false;
}
static void handleSLCAN(const String& s){
  if (!s.length()) return;
  char cmd=s[0];
  if (cmd=='S'){ uint32_t b=slcan_bitrate(s.length()>1?s[1]:'?'); if(b){slcan_baud=b; Serial.print('\r');} else Serial.print('\a'); return; }
  if (cmd=='O'){ Head1.setBaudRate(slcan_baud, TX);          Head1.enableFIFO(); slcan_open=true; Serial.print('\r'); return; }
  if (cmd=='L'){ Head1.setBaudRate(slcan_baud, LISTEN_ONLY); Head1.enableFIFO(); slcan_open=true; Serial.print('\r'); return; }
  if (cmd=='C'){ slcan_open=false; Serial.print('\r'); return; }
  if (cmd=='V'){ Serial.print("V0400\r"); return; }
  if (cmd=='v'){ Serial.print("v0400\r"); return; }
  if (cmd=='N'){ Serial.print("NCB01\r"); return; }
  if (cmd=='F'){ Serial.print("F00\r");   return; }                       // status flags: no errors
  if (cmd=='Z'||cmd=='z'||cmd=='m'||cmd=='M'){ Serial.print('\r'); return; } // timestamp/filter cmds: accept+ignore (we pass all)
  if (cmd=='t'){                                                          // tIIILDD..
    if (s.length()<5){ Serial.print('\a'); return; }
    uint32_t id=(nib(s[1])<<8)|(nib(s[2])<<4)|nib(s[3]);
    int dlc=nib(s[4]); if(dlc>8)dlc=8;
    CAN_message_t m; m.id=id; m.flags.extended=0; m.len=dlc;
    for (int i=0;i<dlc;i++){ int p=5+i*2; if(p+1>=(int)s.length()){Serial.print('\a');return;} m.buf[i]=(nib(s[p])<<4)|nib(s[p+1]); }
    Serial.print((slcan_open && Head1.write(m)) ? "z\r" : "\a"); return;
  }
  if (cmd=='T'){                                                          // TIIIIIIIILDD..
    if (s.length()<10){ Serial.print('\a'); return; }
    uint32_t id=0; for (int i=1;i<=8;i++) id=(id<<4)|nib(s[i]);
    int dlc=nib(s[9]); if(dlc>8)dlc=8;
    CAN_message_t m; m.id=id; m.flags.extended=1; m.len=dlc;
    for (int i=0;i<dlc;i++){ int p=10+i*2; if(p+1>=(int)s.length()){Serial.print('\a');return;} m.buf[i]=(nib(s[p])<<4)|nib(s[p+1]); }
    Serial.print((slcan_open && Head1.write(m)) ? "Z\r" : "\a"); return;
  }
  if (cmd=='r'||cmd=='R'){ Serial.print('\r'); return; }                  // RTR tx: accept (VAG rarely needs it)
  Serial.print('\a');                                                     // unknown SLCAN cmd
}

// ---------------- line protocol ----------------
static int split(const String& s, char sep, String* parts, int maxp){
  int count=0, start=0;
  for (int i=0;i<=(int)s.length() && count<maxp;i++){
    if (i==(int)s.length() || s[i]==sep){ parts[count++]=s.substring(start,i); start=i+1; }
  }
  return count;
}

void handleLine(String line){
  line.trim();
  if (line.length()==0) return;

  String parts[6];
  int np = split(line, ':', parts, 6);
  String kw = parts[0]; kw.toUpperCase();

  if (kw=="PING"){ Serial.println("PONG"); return; }
  if (kw=="SLCAN"){ slcan_mode=true; Serial.println("OK:slcan (Lawicel mode on Head 1; reset board to exit)"); return; }
  if (kw=="INFO"){
    Serial.print("CERBERUS:"); Serial.print(CERBERUS_VERSION);
    Serial.print(" CAN1="); Serial.print(BUS1_BAUD);
    Serial.print(" CAN2="); Serial.print(BUS2_BAUD);
    Serial.print(" tmo="); Serial.print(UDS_TIMEOUT_MS);
    Serial.print(" respmax=4096"); Serial.print(" monring="); Serial.print(MON_RING); Serial.println();
    return;
  }
  if (kw=="STATS"){
    // CAN controller health — error counters + fault state. Use it to see whether a
    // sniff/exchange is dropping frames or the bus is erroring (e.g. ACK/CRC errors).
    int bus = (np>=2)?parts[1].toInt():1;
    if (bus!=1 && bus!=2){ Serial.println("ERR:bus (1|2)"); return; }
    CAN_error_t e;
    if (bus==1) Head1.error(e,false); else Head2.error(e,false);
    Serial.print("STATS:bus="); Serial.print(bus);
    Serial.print(" state="); Serial.print(e.state);
    Serial.print(" fltconf="); Serial.print(e.FLT_CONF);
    Serial.print(" rxerr="); Serial.print(e.RX_ERR_COUNTER);
    Serial.print(" txerr="); Serial.print(e.TX_ERR_COUNTER);
    Serial.print(" ack="); Serial.print(e.ACK_ERR);
    Serial.print(" crc="); Serial.print(e.CRC_ERR);
    Serial.print(" frm="); Serial.print(e.FRM_ERR);
    Serial.print(" stf="); Serial.print(e.STF_ERR);
    Serial.print(" esr1=0x"); Serial.println(e.ESR1, HEX);
    return;
  }
  if (kw=="MON"){
    // MON:on[:idlo:idhi] | MON:off | MON:stat — always-on Head-2 ring-buffered logger.
    // M2:<ms>:<id>:<hex>[:OVR] streams interleaved with command replies; the ring decouples
    // capture from USB so bursts / blocking Head-1 transactions don't drop frames.
    if (np>=2 && parts[1].equalsIgnoreCase("off")){
      mon_on=false;
      Serial.print("OK:mon-off dropped="); Serial.print(mon_dropped);
      Serial.print(" peak="); Serial.println(mon_peak);
      return;
    }
    if (np>=2 && parts[1].equalsIgnoreCase("stat")){
      Serial.print("M2STAT:on="); Serial.print(mon_on?1:0);
      Serial.print(" depth="); Serial.print(mon_depth());
      Serial.print(" peak="); Serial.print(mon_peak);
      Serial.print(" dropped="); Serial.print(mon_dropped);
      Serial.print(" total="); Serial.print(mon_total);
      Serial.print(" cap="); Serial.println(MON_RING);
      return;
    }
    if (np>=2 && parts[1].equalsIgnoreCase("on")){
      mon_idlo = (np>=3)?strtoul(parts[2].c_str(),nullptr,16):0x000;
      mon_idhi = (np>=4)?strtoul(parts[3].c_str(),nullptr,16):0x7FF;
      Head2.setBaudRate(BUS2_BAUD, LISTEN_ONLY); Head2.enableFIFO();
      mon_head=mon_tail=0; mon_dropped=0; mon_peak=0; mon_total=0;   // fresh ring per session
      mon_t0 = millis(); mon_on = true;
      Serial.println("OK:mon-on");
      return;
    }
    Serial.println("ERR:format (MON:on[:idlo:idhi] | MON:off | MON:stat)");
    return;
  }
  if (kw=="SNIFF"){
    if (np<2){ Serial.println("ERR:format (SNIFF:bus:ms[:idlo:idhi])"); return; }
    int bus = parts[1].toInt();
    uint32_t ms   = (np>=3)?(uint32_t)parts[2].toInt():0;
    uint32_t idlo = (np>=4)?strtoul(parts[3].c_str(),nullptr,16):0x000;
    uint32_t idhi = (np>=5)?strtoul(parts[4].c_str(),nullptr,16):0x7FF;
    // Passive sniff in hardware LISTEN-ONLY (LOM): never ACK, never disturb the bus
    // (safe to run alongside ODIS). Restore TX on exit. enableFIFO() is re-asserted
    // after each baud/LOM re-init so reception keeps working.
    if (bus==1){
      Head1.setBaudRate(BUS1_BAUD, LISTEN_ONLY); Head1.enableFIFO();
      do_sniff(Head1, ms, idlo, idhi);
      Head1.setBaudRate(BUS1_BAUD, TX); Head1.enableFIFO();
    } else if (bus==2){
      Head2.setBaudRate(BUS2_BAUD, LISTEN_ONLY); Head2.enableFIFO();
      do_sniff(Head2, ms, idlo, idhi);
      Head2.setBaudRate(BUS2_BAUD, TX); Head2.enableFIFO();
    } else Serial.println("ERR:bus (1|2)");
    return;
  }
  if (kw=="TP"){
    // background, non-blocking:  TP:bus:TX:ms  starts ;  TP:STOP (or TP:0) stops.
    if (np>=2 && (parts[1].equalsIgnoreCase("STOP") || parts[1]=="0")){ tp_bus=0; Serial.println("OK:tp-off"); return; }
    if (np<4){ Serial.println("ERR:format (TP:bus:TX:ms | TP:STOP)"); return; }
    int bus = parts[1].toInt();
    if (bus!=1 && bus!=2){ Serial.println("ERR:bus (1|2)"); return; }
    tp_bus = bus;
    tp_tx  = strtoul(parts[2].c_str(), nullptr, 16);
    tp_ms  = (uint32_t)parts[3].toInt(); if (tp_ms<20) tp_ms=20;
    tp_last = 0;
    Serial.print("OK:tp-on bus="); Serial.print(tp_bus);
    Serial.print(" tx="); Serial.print(tp_tx,HEX);
    Serial.print(" ms="); Serial.println(tp_ms);
    return;
  }
  if (kw=="SCAN"){
    int bus = (np>=2)?parts[1].toInt():1;
    uint32_t lo = (np>=3)?strtoul(parts[2].c_str(),nullptr,16):0x700;
    uint32_t hi = (np>=4)?strtoul(parts[3].c_str(),nullptr,16):0x7EF;
    uint32_t win= (np>=5)?(uint32_t)parts[4].toInt():120;
    if (bus==1) do_scan(Head1, lo, hi, win);
    else if (bus==2) do_scan(Head2, lo, hi, win);
    else Serial.println("ERR:bus (1|2)");
    return;
  }
  if (kw=="RAW"){
    if (np<4){ Serial.println("ERR:format (RAW:bus:ID:HEX)"); return; }
    int bus = parts[1].toInt();
    uint32_t id = strtoul(parts[2].c_str(), nullptr, 16);
    uint8_t d[8]; int n = hexBytes(parts[3], d, sizeof(d));
    if (n<0){ Serial.println("ERR:hex (>8 bytes / odd)"); return; }
    if (bus==1) do_raw(Head1, id, d, n);
    else if (bus==2) do_raw(Head2, id, d, n);
    else Serial.println("ERR:bus (1|2)");
    return;
  }
  if (kw=="CANX"){
    // CANX:bus:ID:HEX[:waitms]  — send one classic frame (TX mode) then listen waitms ms.
    // RX frames print as  RX:<id>:<hex>  then  DONE:<n>  (waitms>0), or OK:sent (waitms==0).
    if (np<4){ Serial.println("ERR:format (CANX:bus:ID:HEX[:waitms])"); return; }
    int bus = parts[1].toInt();
    uint32_t id = strtoul(parts[2].c_str(), nullptr, 16);
    uint8_t d[8]; int n = hexBytes(parts[3], d, sizeof(d));
    if (n<0){ Serial.println("ERR:hex (>8 bytes / odd)"); return; }
    uint32_t waitms = (np>=5)?(uint32_t)parts[4].toInt():0;
    if (bus==1) do_canx(Head1, id, d, n, waitms);
    else if (bus==2) do_canx(Head2, id, d, n, waitms);
    else Serial.println("ERR:bus (1|2)");
    return;
  }
  if (kw=="UDS"){
    if (np<5){ Serial.println("ERR:format (UDS:bus:TX:RX:HEX)"); return; }
    int bus = parts[1].toInt();
    uint32_t tx = strtoul(parts[2].c_str(), nullptr, 16);
    uint32_t rx = strtoul(parts[3].c_str(), nullptr, 16);
    static uint8_t req[520]; int reqlen = hexBytes(parts[4], req, sizeof(req));
    if (reqlen<=0){ Serial.println("ERR:hex (too long / odd)"); return; }
    if (bus==1) do_uds(Head1, tx, rx, req, reqlen);
    else if (bus==2) do_uds(Head2, tx, rx, req, reqlen);
    else Serial.println("ERR:bus (1|2)");
    return;
  }

  // shorthand:  TXID:RXID:HEX  -> Head 1
  if (np==3){
    uint32_t tx = strtoul(parts[0].c_str(), nullptr, 16);
    uint32_t rx = strtoul(parts[1].c_str(), nullptr, 16);
    static uint8_t req[520]; int reqlen = hexBytes(parts[2], req, sizeof(req));
    if (reqlen<=0){ Serial.println("ERR:hex (too long / odd)"); return; }
    do_uds(Head1, tx, rx, req, reqlen);
    return;
  }

  Serial.println("ERR:unknown (PING|INFO|SCAN|SNIFF|MON|STATS|TP|RAW|CANX|UDS|SLCAN|TX:RX:HEX)");
}

void setup(){
  Serial.begin(115200);
  Head1.begin(); Head1.setBaudRate(BUS1_BAUD); Head1.setMaxMB(16); Head1.enableFIFO();
  Head2.begin(); Head2.setBaudRate(BUS2_BAUD, LISTEN_ONLY); Head2.setMaxMB(16); Head2.enableFIFO();
  oled_init();
}

String inbuf;
void loop(){
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\n' || c=='\r'){
      if (inbuf.length()){
        if (slcan_mode)            handleSLCAN(inbuf);                   // already in SLCAN mode
        else if (is_slcan_cmd(inbuf)) { slcan_mode = true; handleSLCAN(inbuf); }  // auto-enter
        else                       handleLine(inbuf);                   // native protocol
        inbuf = "";
      }
    }
    else if (inbuf.length() < 1100) inbuf += c;       // room for a full TransferData chunk
  }
  if (slcan_mode){                                    // SLCAN: stream RX frames out as t/T lines
    if (slcan_open){ CAN_message_t r; while (Head1.read(r)) slcan_emit(r); }
  } else {
    pump_mon();                                       // always-on Head-2 background logger
    tp_service();                                     // native: non-blocking TesterPresent keep-alive
  }
  oled_update();                                      // throttled OLED HUD (no-op if no panel)
}
