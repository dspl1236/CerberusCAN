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
 *   EMU:on:bus:REQ:RESP            emulate a module on Head 1 (UDS responder) -> EMURX:/EMUTX:
 *   EMU:add:PREFIX:RESP            add a rule (request-prefix -> response); EMU:clear|off|stat
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
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Head2;   // 2nd HVD230 on the SAME diag bus, 500k (OBD 6/14) — listen-only logger
// FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> Head3;  // CAN-FD (pins 30/31), classic-CAN C7 never uses it.

// Live bus rates. They start at the cerberus_config.h defaults and are retunable at runtime
// with BAUD:<bus>:<rate>, because a head moved onto ANOTHER bus is usually another rate --
// the COMFORT CAN, for one, is tapped off the gateway (it is not on the OBD socket at all on
// a gateway car) and does not run at the diagnostic bus's 500k. Deliberately RAM-only: they
// revert to the defaults on reboot, so an experiment can never leave the board mysteriously
// mistuned the way a persisted setting would.
static uint32_t bus1_baud = BUS1_BAUD;
static uint32_t bus2_baud = BUS2_BAUD;

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
static void printHexTo(Print& o,const uint8_t* d,int n){ for(int i=0;i<n;i++){ if(d[i]<16)o.print('0'); o.print(d[i],HEX);} }
static void printHex(const uint8_t* d,int n){ printHexTo(Serial,d,n); }

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
#define OLED_CELLS 12
static uint8_t   oled_addr = 0;
SSD1306AsciiWire oled;
static bool      oled_present = false;
static uint32_t  oled_last = 0;
static uint32_t  h2_prev = 0, h1_prev = 0, h2_pk = 1, h1_pk = 1;   // VU auto-scale state
static uint32_t  h1_cmd   = 0;               // active-command count -> the H1 "our activity" VU bar
static char      mode_buf[10] = {0};         // last command keyword, flashed in the title
static const char* cur_mode_name = "VCI";    // VCI | SNIFF | DUAL — steady OLED title + MODE report
static uint32_t  mode_ts = 0;
static void set_mode(const char* m){ uint8_t i=0; for(; m[i] && i<9; i++) mode_buf[i]=m[i]; mode_buf[i]=0; mode_ts=millis(); }

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

static void oled_bar(uint8_t row, const char* lbl, int cells){
  if (cells > OLED_CELLS) cells = OLED_CELLS; if (cells < 0) cells = 0;
  oled.setCursor(0, row); oled.print(lbl); oled.print(" [");
  for (int i=0;i<OLED_CELLS;i++) oled.print(i<cells ? '#' : ' ');
  oled.print(']'); oled.clearToEOL();
}

static void oled_update(){
  if (!oled_present) return;
  uint32_t now = millis();
  if (now - oled_last < 200) return;                  // 5 Hz, never per-loop (I2C is blocking)
  oled_last = now;

  // per-tick deltas -> auto-scaled VU bars (peak rises instantly, decays slowly = "breathing")
  uint32_t h2d = mon_total - h2_prev; h2_prev = mon_total;   // Head 2 = bus frames logged
  uint32_t h1d = h1_cmd    - h1_prev; h1_prev = h1_cmd;      // Head 1 = our active commands
  if (h2d > h2_pk) h2_pk = h2d; else if (h2_pk > 1) h2_pk -= (h2_pk>>4)+1;
  if (h1d > h1_pk) h1_pk = h1d; else if (h1_pk > 1) h1_pk -= (h1_pk>>4)+1;
  int h2c = (int)((uint64_t)h2d * OLED_CELLS / (h2_pk?h2_pk:1));
  int h1c = (int)((uint64_t)h1d * OLED_CELLS / (h1_pk?h1_pk:1));

  // title shows the live MODE (last command, else persistent MON/IDLE) — no version clutter
  const char* mode = (mode_buf[0] && now-mode_ts < 1500) ? mode_buf : cur_mode_name;

  oled.setCursor(0, 0); oled.print("CERBERUS  "); oled.print(mode); oled.clearToEOL();  // yellow band
  oled.setCursor(0, 2); oled.print("tot "); oled.print(mon_total);
                        oled.print(" drp "); oled.print(mon_dropped); oled.clearToEOL();
  oled.setCursor(0, 3); oled.print("peak "); oled.print(mon_peak); oled.clearToEOL();
  oled_bar(5, "H1", h1c);     // our command / TX activity
  oled_bar(6, "H2", h2c);     // bus traffic (logged frames)
}

// Where the M2: log goes. 1 = the command port, interleaved with command replies (a host that
// does not demux "M2:" will read a log line where it expects OK: and appear to hang). 2 = the
// SECOND USB CDC, i.e. a dedicated firehose.
//
// Port 2 exists because Windows opens a COM port EXCLUSIVELY: while a host tool holds the command
// port, no separate monitor app can open it too. Splitting the log onto the other CDC is the only
// way to have someone watching the bus at the same time as a tool driving it -- command/response
// stays clean on port 1, the log streams on port 2, and neither has to demux the other.
//
// The cost: that CDC is normally the always-dumb K-line KKL cable, so while port 2 is selected the
// K-line bridge stands down (see kline_bridge()).
static uint8_t mon_port = 1;
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  #define MON_HAS_PORT2 1
  #define MON_OUT   (mon_port==2 ? (Print&)SerialUSB1 : (Print&)Serial)
  #define MON_ROOM  (mon_port==2 ? SerialUSB1.availableForWrite() : Serial.availableForWrite())
#else
  #define MON_HAS_PORT2 0
  #define MON_OUT   ((Print&)Serial)
  #define MON_ROOM  (Serial.availableForWrite())
#endif

static void mon_flush(uint32_t maxframes){            // ring -> USB (only when TX has room)
  uint32_t sent=0;
  while (!mon_empty() && sent<maxframes){
    if ((uint32_t)MON_ROOM < 40) break;               // would block -> stop; ring holds it
    Print &o = MON_OUT;
    MonFrame &f = mon_ring[mon_tail];
    o.print("M2:"); o.print(f.t); o.print(':');
    o.print(f.id, HEX); o.print(':');
    printHexTo(o, f.buf, f.len);
    if (f.ovr) o.print(":OVR");
    o.println();
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
// CANX = send-then-listen, so the host can drive low-level request/response protocols
// (bootloaders, raw probing, custom framing) entirely from a script — no per-tweak reflash.
// waitms==0 => fire-and-forget (fast streaming).
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
// Drop whatever the FIFO/mailboxes already hold before a capture starts counting.
//
// EVERY re-init of a head (setBaudRate + enableFIFO) leaves stale mailbox entries that read
// back as id 0 / len 0. On Head 2 that surfaced as exactly 6 phantom "RX:0:0:" lines at t=0
// on every capture -- the same 6 at a 2000 ms and a 5000 ms window, which is what gives them
// away as drained garbage rather than bus traffic.
//
// Drain for a settle WINDOW, deliberately NOT "until the first empty read": the stale entries
// do not surface the instant enableFIFO() returns, so breaking on the first empty read exits
// too early and leaves them to appear inside the capture. Bounded by both a time window and a
// frame count so a genuinely busy bus can never stall here. Costs the first few ms of a
// capture, which is the right trade for a logger whose whole job is to be trusted.
//
// Shared by BOTH capture paths -- SNIFF and MON:on. Fixing only do_sniff left MON:on emitting
// the very same 6 phantom "M2:0:0:" frames, so it lives here where neither can forget it.
template <typename BUS>
static void drain_stale(BUS& bus){
  CAN_message_t junk;
  uint32_t t0 = millis(), n = 0;
  while ((millis() - t0) < SNIFF_SETTLE_MS && n < 512){
    if (bus.read(junk)) n++;
  }
}

template <typename BUS>
static void do_sniff(BUS& bus, uint32_t ms, uint32_t idlo, uint32_t idhi){
  drain_stale(bus);
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
  uint32_t found = 0, filtered = 0;
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
        // Count only a REAL answer to the TesterPresent we just sent: a single frame carrying
        // either the positive 7E or a negative 7F 3E. ANY frame used to count, which made the
        // sweep report phantoms on a bus that carries traffic -- and worse, a FUNCTIONAL address
        // (0x700, 0x7DF) is answered by every module at once, so those replies drained into the
        // windows of the ids that came after it. A 700-7DF sweep claimed 45 modules where 26
        // exist, the extras being a contiguous block right after 0x700. Keep waiting inside the
        // window rather than giving up, so a genuine reply still lands behind unrelated traffic.
        uint8_t pci = r.buf[0] >> 4;
        bool real = (pci == 0) && r.len >= 3 &&
                    (r.buf[1] == 0x7E || (r.buf[1] == 0x7F && r.buf[2] == 0x3E));
        if (!real){ filtered++; continue; }
        Serial.print("FOUND:"); Serial.print(tx, HEX); Serial.print(':');
        Serial.print(r.id, HEX); Serial.print(':');
        printHex(r.buf, r.len); Serial.println();
        found++;
        break;
      }
    }
  }
  Serial.print("DONE:"); Serial.print(found);
  if (filtered){ Serial.print(" filtered:"); Serial.print(filtered); }  // unrelated frames seen
  Serial.println();
}

// ---------------- EMU: module emulation (UDS responder) ----------------
// Cerberus answers UDS requests on emu_req from emu_resp, to probe how a tester / the J533
// gateway reacts to a FAKE module. Rules map a request *prefix* -> a canned response:
// "10" matches any session-control request, "22F190" a ReadDID F190, "27" any SecurityAccess.
// First matching rule wins; unmatched requests get a negative response (7F <sid> 11) by default.
// Reassembles multi-frame requests (sends flow control) and multi-frame responses (isotp_send).
// Runs on Head 1 (it must ACK + transmit) -> a BENCH play (gateway + fake module), NOT a tap
// alongside the real module. Each handled exchange is echoed to the host as EMURX:/EMUTX:.
struct EmuRule { uint8_t plen; uint8_t rlen; uint8_t prefix[8]; uint8_t resp[64]; };
#define EMU_RULES 24
static EmuRule  emu_rules[EMU_RULES];
static int      emu_nrules = 0;
static bool     emu_on  = false;
static uint32_t emu_req = 0, emu_resp = 0, emu_hits = 0;

// Receive a full ISO-TP request addressed to emu_req on Head 1. Returns length, or 0 if none ready.
static int emu_recv(uint8_t* out, int maxlen){
  CAN_message_t r;
  if (!Head1.read(r) || r.id != emu_req) return 0;
  uint8_t pci = r.buf[0]>>4;
  if (pci==0){                                       // single frame
    int n = r.buf[0]&0x0F; if (n>maxlen) n=maxlen;
    for (int i=0;i<n;i++) out[i]=r.buf[1+i];
    return n;
  }
  if (pci==1){                                       // first frame -> send CTS, collect CFs
    int total=((r.buf[0]&0x0F)<<8)|r.buf[1]; if (total>maxlen) total=maxlen;
    int got=0; for (int i=0;i<6 && got<total;i++) out[got++]=r.buf[2+i];
    CAN_message_t fc; fc.id=emu_resp; fc.flags.extended=0; fc.len=8;
    for (int i=0;i<8;i++) fc.buf[i]=PAD_BYTE;
    fc.buf[0]=0x30; fc.buf[1]=0; fc.buf[2]=0;         // CTS, BS=0, STmin=0
    Head1.write(fc);
    uint32_t dl=millis()+1000;
    while (got<total && (int32_t)(dl-millis())>0){
      pump_mon();                                     // keep the Head-2 logger draining
      CAN_message_t c;
      if (Head1.read(c) && c.id==emu_req && (c.buf[0]>>4)==2){
        for (int i=0;i<7 && got<total;i++) out[got++]=c.buf[1+i];
        dl=millis()+1000;
      }
    }
    return got;
  }
  return 0;                                           // stray FC/CF
}

static void emu_service(){
  if (!emu_on) return;
  static uint8_t req[64];
  int n = emu_recv(req, sizeof(req));
  if (n<=0) return;
  emu_hits++;
  Serial.print("EMURX:"); printHex(req, n); Serial.println();
  for (int i=0;i<emu_nrules;i++){
    EmuRule& rl = emu_rules[i];
    if (n < rl.plen) continue;
    bool m=true; for (int j=0;j<rl.plen;j++) if (req[j]!=rl.prefix[j]){ m=false; break; }
    if (m){
      Serial.print("EMUTX:"); printHex(rl.resp, rl.rlen); Serial.println();
      isotp_send(Head1, emu_resp, emu_req, rl.resp, rl.rlen);
      return;
    }
  }
  uint8_t neg[3] = { 0x7F, req[0], 0x11 };            // default: serviceNotSupported
  Serial.print("EMUTX:"); printHex(neg, 3); Serial.println();
  isotp_send(Head1, emu_resp, emu_req, neg, 3);
}

// ---------------- SLCAN (Lawicel ASCII) ----------------
// A standard serial-CAN protocol so the board drops into the wider tool ecosystem:
// SavvyCAN (direct), python-can (slcan), and slcand -> SocketCAN -> Wireshark/CANdevStudio.
// Single channel on Head 1 (the 500k diagnostic bus). Replies: '\r' ok, '\a' (BEL) error.
// Frames stream out as t<iii><l><dd..> (standard) / T<iiiiiiii><l><dd..> (extended).
bool slcan_mode = false;
static bool slcan_open = false;
static uint32_t slcan_baud = bus1_baud;

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
  auto ishex=[](char x){ return (x>='0'&&x<='9')||(x>='a'&&x<='f')||(x>='A'&&x<='F'); };
  // Data/remote frames must carry real hex ID+len digits, else text commands that merely START
  // with t/T/r/R (TP, TX:, RAW:) would be mistaken for SLCAN and silently flip the board into
  // SLCAN mode (recoverable only by reset). t/r = 3-hex ID + len ; T/R = 8-hex ID + len.
  if (c=='t'||c=='r'){ if (s.length()<5)  return false; for(int i=1;i<=4;i++) if(!ishex(s[i])) return false; return true; }
  if (c=='T'||c=='R'){ if (s.length()<10) return false; for(int i=1;i<=9;i++) if(!ishex(s[i])) return false; return true; }
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

// ---------------- VAG TP2.0 (Transport Protocol 2.0) ----------------
//
// The comfort-domain modules (doors, seat memory) are NOT reachable with UDS-over-ISO-TP.
// They sit on the comfort CAN behind the gateway and speak KWP2000 over TP2.0, whose ids
// live at 0x200 / 0x2xx / 0x3xx / 0x4Cx -- entirely outside the 0x700-0x7DF diagnostic
// range. An exhaustive ISO-TP sweep of that range finds nothing, which is exactly why they
// looked unreachable and why a comfort-bus tap looked mandatory. It isn't: the gateway
// routes TP2.0 onto the ordinary OBD 6/14 pair.
//
// Framing (captured from a dealer tool on a 958.2, see docs):
//   setup    -> 0x200        : <dest> C0 00 10 <ourRx_lo> <ourRx_hi> 01
//   reply    <- 0x200+dest   : 00 D0 <ourRx echoed> <ecuTx_lo> <ecuTx_hi> 01
//   params   -> ecuTx        : A0 0F 8A FF 00 FF        (<- A1 ... is the reply)
//   data     : <op|seq> [len_hi len_lo] payload
//              op nibble 1 = last of block, ACK expected ; 2 = more follows, no ACK
//              ONLY the first PDU of a message carries the 16-bit length
//   ack      : B<nextSeq>          keepalive : A3 (-> A1)          close : A8
// Sequence numbers are per-direction and run continuously ACROSS messages, not per message.
// Frames are sent unpadded, exactly as the dealer tool does.

#define TP20_SETUP_ID     0x200
#define TP20_KEEPALIVE_MS 900          // the channel dies at ~1 s of silence
static int      tp20_bus  = 0;         // 0 = no channel open, else 1|2
static uint8_t  tp20_dest = 0;
static uint16_t tp20_rxid = 0;         // ECU -> us
static uint16_t tp20_txid = 0;         // us -> ECU
static uint8_t  tp20_seq  = 0;         // our next tx sequence nibble
static uint32_t tp20_ka   = 0;         // last keepalive sent
static bool     tp20_busy = false;     // suppress the background keepalive mid-transaction

static bool tp20_w(uint16_t id, const uint8_t* d, int n){
  CAN_message_t m; m.id = id; m.flags.extended = 0; m.len = n;
  for (int i = 0; i < n; i++) m.buf[i] = d[i];
  return (tp20_bus == 2) ? Head2.write(m) : Head1.write(m);
}
static bool tp20_r(CAN_message_t& r){
  return (tp20_bus == 2) ? Head2.read(r) : Head1.read(r);
}
static bool tp20_await(uint16_t id, uint8_t* out, int& n, uint32_t ms){
  uint32_t dl = millis() + ms;
  while ((int32_t)(dl - millis()) > 0){
    CAN_message_t r;
    if (tp20_r(r) && r.id == id){
      n = r.len;
      for (int i = 0; i < r.len; i++) out[i] = r.buf[i];
      return true;
    }
  }
  return false;
}

static void tp20_disconnect(){
  if (tp20_bus){ uint8_t a = 0xA8; tp20_w(tp20_txid, &a, 1); }
  tp20_bus = 0; tp20_dest = 0; tp20_rxid = tp20_txid = 0; tp20_seq = 0;
}

static bool tp20_connect(int bus, uint8_t dest, uint16_t ourRx){
  tp20_bus = bus;
  uint8_t req[7] = { dest, 0xC0, 0x00, 0x10,
                     (uint8_t)(ourRx & 0xFF), (uint8_t)((ourRx >> 8) & 0x0F), 0x01 };
  if (!tp20_w(TP20_SETUP_ID, req, 7)){ tp20_bus = 0; return false; }
  uint8_t b[8]; int n = 0;
  if (!tp20_await(TP20_SETUP_ID + dest, b, n, 600)){ tp20_bus = 0; return false; }
  if (n < 7 || b[1] != 0xD0){ tp20_bus = 0; return false; }      // D0 = setup accepted
  tp20_rxid = ourRx;
  tp20_txid = ((b[5] & 0x0F) << 8) | b[4];                       // the id the ECU chose
  uint8_t par[6] = { 0xA0, 0x0F, 0x8A, 0xFF, 0x00, 0xFF };       // mirror the dealer tool
  tp20_w(tp20_txid, par, 6);
  tp20_await(tp20_rxid, b, n, 400);                              // A1; not fatal if missed
  tp20_dest = dest; tp20_seq = 0; tp20_ka = millis();
  return true;
}

static bool tp20_send(const uint8_t* p, int len){
  int off = 0; bool first = true;
  while (off < len){
    uint8_t f[8]; int k = 0;
    int avail = 7 - (first ? 2 : 0);
    int chunk = len - off; if (chunk > avail) chunk = avail;
    bool last = (off + chunk >= len);
    f[k++] = (last ? 0x10 : 0x20) | (tp20_seq & 0x0F);
    if (first){ f[k++] = (len >> 8) & 0xFF; f[k++] = len & 0xFF; first = false; }
    for (int i = 0; i < chunk; i++) f[k++] = p[off + i];
    off += chunk;
    if (!tp20_w(tp20_txid, f, k)) return false;
    tp20_seq = (tp20_seq + 1) & 0x0F;
    if (last){                                    // op 1x asked for it: wait for B<seq>
      uint8_t b[8]; int n = 0;
      if (!tp20_await(tp20_rxid, b, n, 300)) return false;
    }
  }
  return true;
}

static int tp20_recv(uint8_t* out, int cap, uint32_t ms){
  int want = -1, got = 0;
  uint32_t dl = millis() + ms;
  while ((int32_t)(dl - millis()) > 0){
    CAN_message_t r;
    if (!tp20_r(r) || r.id != tp20_rxid || r.len < 1) continue;
    uint8_t op = r.buf[0] >> 4;
    if (op == 0xA || op == 0xB) continue;                 // params / ack / keepalive
    int i = 1;
    if (want < 0){
      if (r.len < 3) continue;
      want = (r.buf[1] << 8) | r.buf[2];
      i = 3;
    }
    for (; i < r.len && got < cap; i++) out[got++] = r.buf[i];
    if (op == 0x0 || op == 0x1){                          // ACK expected
      uint8_t a = 0xB0 | ((r.buf[0] + 1) & 0x0F);
      tp20_w(tp20_txid, &a, 1);
    }
    if (want >= 0 && got >= want) return want;
    dl = millis() + ms;                                   // keep extending while it flows
  }
  return -1;
}

// Background keepalive: without an A3 roughly every second the ECU drops the channel.
static void tp20_service(){
  if (!tp20_bus || tp20_busy) return;
  if ((uint32_t)(millis() - tp20_ka) < TP20_KEEPALIVE_MS) return;
  uint8_t a = 0xA3;
  tp20_w(tp20_txid, &a, 1);
  tp20_ka = millis();
}

// ---------------- line protocol ----------------
static int split(const String& s, char sep, String* parts, int maxp){
  int count=0, start=0;
  for (int i=0;i<=(int)s.length() && count<maxp;i++){
    if (i==(int)s.length() || s[i]==sep){ parts[count++]=s.substring(start,i); start=i+1; }
  }
  return count;
}

// ---------------- K-line / KWP2000 — "Head 3" for pre-CAN VAG (ISO 14230 / ISO 9141) ----------------
// K-line is a single-wire 12V UART bus (OBD pin 7), NOT CAN — so it rides a spare hardware UART
// (Serial2: RX 7, TX 8) through a K-line transceiver (TJA1021 / MC33660 / L9637D; see HARDWARE.md).
// Half-duplex: our own TX echoes back on RX, so every send swallows the echo.
// BENCH-UNTESTED — needs the transceiver + a pre-CAN VAG on hand. Protocol per ISO 14230-2/-3.
#define KL Serial2
static const int      KL_TX = 8, KL_RX = 7;
static uint32_t kl_baud = 10400;              // K-line UART baud — 10400 default; 9600 for M232/AAN (KWP:baud:9600)
static bool    kl_up  = false;
static uint8_t kl_tgt = 0x01;                 // ECU address (engine = 0x01, per KWPBridge), set by init
static bool    kl_passthru = false;           // transparent "dumb KKL cable" mode (e.g. for NefMoto) — reset to exit

static uint8_t kl_cks(const uint8_t* b, int n){ uint16_t s=0; for(int i=0;i<n;i++) s+=b[i]; return (uint8_t)s; }

static void kl_send(const uint8_t* b, int n){ // write bytes, swallowing the half-duplex echo
  while (KL.available()) KL.read();
  for (int i=0;i<n;i++){
    KL.write(b[i]); KL.flush();
    uint32_t t=millis(); while(millis()-t<25){ if(KL.available()){ KL.read(); break; } }
  }
}

static int kl_recv(uint8_t* buf, int max, uint32_t window){  // burst read w/ inter-byte gap
  int n=0; uint32_t t=millis(); uint32_t w=window;
  while (millis()-t < w && n < max){
    if (KL.available()){ buf[n++]=KL.read(); t=millis(); w=60; }   // shrink to inter-byte gap after 1st
  }
  return n;
}

static void kl_dump(const char* tag, const uint8_t* r, int n){
  Serial.print(tag);
  for(int i=0;i<n;i++){ if(r[i]<16) Serial.print('0'); Serial.print(r[i],HEX); }
  Serial.println(n? "" : "(no response)");
}

static void kl_fastinit(uint8_t target){      // ISO 14230 fast init = line wiggle only; host then sends the session
  kl_tgt=target; KL.end(); pinMode(KL_TX,OUTPUT);
  digitalWrite(KL_TX,HIGH); delay(300);       // idle high
  digitalWrite(KL_TX,LOW);  delay(25);        // 25ms low
  digitalWrite(KL_TX,HIGH); delay(25);        // 25ms high
  KL.begin(kl_baud); delay(300);              // ECU sync wait (per KWPBridge KWP2000_FAST_INIT_WAIT)
  while(KL.available()) KL.read();
  kl_up=true;
  Serial.println("OK:kwp-fast-ready (now send the session: KWP:1089 for ME7, or KWP:81 StartComm)");
}

static void kl_slowinit(uint8_t addr){        // 5-baud init: bit-bang the addr @5 baud -> sync 0x55 + keybytes
  KL.end(); pinMode(KL_TX,OUTPUT);
  digitalWrite(KL_TX,HIGH); delay(300);
  digitalWrite(KL_TX,LOW);  delay(200);                       // start bit
  for(int i=0;i<8;i++){ digitalWrite(KL_TX,(addr>>i)&1); delay(200); }   // 8 data bits, LSB first
  digitalWrite(KL_TX,HIGH); delay(200);                       // stop bit
  KL.begin(kl_baud);
  uint8_t r[24]; int n=kl_recv(r,24,600); kl_up=(n>0);
  kl_dump("OK:kwp-slowinit ", r, n);
}

static void kl_kwp(const uint8_t* data, int len){  // framed KWP2000 request -> response payload (SID+data)
  if (!kl_up){ Serial.println("ERR:kwp-not-initialized (KWP:fast or KWP:slow first)"); return; }
  // ISO 14230 frame (per KWPBridge): [fmt=0x80][target][source=0xF1][len][sid+data][checksum]
  uint8_t msg[264]; int m=0;
  msg[m++]=0x80; msg[m++]=kl_tgt; msg[m++]=0xF1; msg[m++]=(uint8_t)len;
  for(int i=0;i<len && m<262;i++) msg[m++]=data[i];
  msg[m]=kl_cks(msg,m); m++;
  kl_send(msg,m);
  uint8_t r[264]; int n=kl_recv(r,264,500);
  if(n<4){ Serial.println("ERR:no-response"); return; }
  int plen=r[3], avail=n-5;                    // response: [fmt][tgt][src][len][payload..][cks]
  if(plen>avail) plen=(avail<0)?0:avail;        // clamp to what actually arrived
  Serial.print("OK:");                          // strip header + checksum -> response payload only
  for(int i=0;i<plen;i++){ if(r[4+i]<16)Serial.print('0'); Serial.print(r[4+i],HEX); }
  Serial.println();
}

// ---- Dual-USB raw K-line bridge: SerialUSB1 (the 2nd CDC port) is an ALWAYS-DUMB KKL cable.
// When a host app (NefMoto / VCDS-dumb / etc.) opens that 2nd COM port, Cerberus transparently
// bridges raw bytes USB2<->K-line (Serial2) — no framing, no echo-swallow, the host owns the
// protocol. The smart KWP/KW1281 path on the PRIMARY port yields the K-line while this is active
// and reclaims it automatically when the 2nd port closes (no replug). No buffer needed: K-line is
// ~10.4 kbaud half-duplex and USB CDC backpressures losslessly; writes are non-blocking so a slow
// K-line drain never stalls loop()/the CAN logger. ----
static bool kl_ext_owned = false;               // true while the 2nd USB port owns the K-line
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
static bool kl_bridge_on = false;
static void kline_bridge(){
  // MON owns the 2nd CDC while mon_port==2: stand down entirely, or K-line bytes would be
  // spliced into the M2: log and a monitor app's keystrokes would be injected onto the K-line.
  if (mon_port == 2){
    if (kl_bridge_on){ kl_bridge_on = false; kl_ext_owned = false; }   // hand the K-line back
    return;
  }
  bool host = (bool)SerialUSB1;                 // 2nd CDC port open by the host (DTR asserted)?
  if (host && !kl_bridge_on){                   // just opened -> claim the K-line for the raw bridge
    KL.end(); KL.begin(kl_baud); while(KL.available()) KL.read();
    kl_bridge_on = true; kl_ext_owned = true;
  } else if (!host && kl_bridge_on){            // just closed -> hand the K-line back to smart KWP
    kl_bridge_on = false; kl_ext_owned = false;
  }
  if (!kl_bridge_on) return;
  for (int n = SerialUSB1.available(); n > 0 && KL.availableForWrite() > 0; --n)
    KL.write(SerialUSB1.read());                // USB2 -> K-line  (non-blocking)
  for (int m = KL.available(); m > 0 && SerialUSB1.availableForWrite() > 0; --m)
    SerialUSB1.write(KL.read());                // K-line -> USB2  (non-blocking)
}
#else
static void kline_bridge(){}                    // single-serial build: no 2nd port, no-op
#endif

// ---- KW1281 (older VAG block protocol) — Head 3 byte engine. Timing-critical -> must run on the
// Teensy (a host-over-USB round-trip per byte is too slow for the complement-ACK). Implemented fresh
// per the protocol (NOT copied from KWPBridge, which stubs the ACK). BENCH-UNTESTED; delays need
// tuning against a real KW1281 ECU. Block = [len][counter][title][data..][0x03]; every byte is
// complement-acked by the other side EXCEPT the 0x03 terminator. ----
static bool    k81_up  = false;
static uint8_t k81_cnt = 0;                    // shared block counter (ECU sets initial; ++ per block)

static bool k81_wb(uint8_t b, bool ack){       // write byte (swallow echo); if ack, await + verify ~b
  while(KL.available()) KL.read();
  KL.write(b); KL.flush();
  uint32_t t=millis(); while(millis()-t<25){ if(KL.available()){ KL.read(); break; } }   // own echo
  if(!ack) return true;
  t=millis(); while(millis()-t<300){ if(KL.available()) return KL.read()==(uint8_t)(~b); }
  return false;
}
static int k81_rb(bool ack, uint32_t to){      // read byte from ECU; if ack, send ~b back (swallow echo)
  uint32_t t=millis();
  while(millis()-t<to){
    if(KL.available()){
      uint8_t b=KL.read();
      if(ack){ delay(5); KL.write((uint8_t)(~b)); KL.flush();
        uint32_t e=millis(); while(millis()-e<25){ if(KL.available()){ KL.read(); break; } } }
      return b;
    }
  }
  return -1;
}
static bool k81_tx(uint8_t title, const uint8_t* data, int n){   // send a block; 0x03 not acked
  if(!k81_wb((uint8_t)(n+3),true)) return false;                 // len = counter+title+data+ETX
  k81_cnt++;
  if(!k81_wb(k81_cnt,true)) return false;
  if(!k81_wb(title,true)) return false;
  for(int i=0;i<n;i++) if(!k81_wb(data[i],true)) return false;
  return k81_wb(0x03,false);
}
static int k81_rx(uint8_t* data, int maxn, uint8_t* title){      // receive a block -> data length
  int len=k81_rb(true,500); if(len<0) return -1;
  int cnt=k81_rb(true,300); if(cnt<0) return -1; k81_cnt=(uint8_t)cnt;
  int ttl=k81_rb(true,300); if(ttl<0) return -1; *title=(uint8_t)ttl;
  int ndata=len-3, got=0;
  for(int i=0;i<ndata;i++){ int b=k81_rb(true,300); if(b<0) return -1; if(got<maxn) data[got++]=(uint8_t)b; }
  k81_rb(false,300);                                             // ETX 0x03 — read, not acked
  return got;
}
static void k81_dump(const char* tag, uint8_t title, const uint8_t* d, int n){
  Serial.print(tag); Serial.print("title="); if(title<16)Serial.print('0'); Serial.print(title,HEX);
  Serial.print(" data=");
  for(int i=0;i<n;i++){ if(d[i]<16)Serial.print('0'); Serial.print(d[i],HEX); }
  Serial.println();
}
static void k81_init(uint8_t addr){            // 5-baud addr -> sync 0x55 + KB1 + KB2 -> reply ~KB2
  KL.end(); pinMode(KL_TX,OUTPUT);
  digitalWrite(KL_TX,HIGH); delay(300);
  digitalWrite(KL_TX,LOW);  delay(200);
  for(int i=0;i<8;i++){ digitalWrite(KL_TX,(addr>>i)&1); delay(200); }
  digitalWrite(KL_TX,HIGH); delay(200);
  KL.begin(kl_baud);
  int sync=-1,kb1=-1,kb2=-1; uint32_t t=millis();
  while(millis()-t<700 && sync<0){ if(KL.available()) sync=KL.read(); }
  t=millis(); while(millis()-t<300 && kb1<0){ if(KL.available()) kb1=KL.read(); }
  t=millis(); while(millis()-t<300 && kb2<0){ if(KL.available()) kb2=KL.read(); }
  if(sync!=0x55 || kb2<0){ Serial.print("ERR:kw1281-init (sync=0x"); Serial.print(sync,HEX); Serial.println(")"); k81_up=false; return; }
  delay(30);
  KL.write((uint8_t)(~kb2)); KL.flush();
  uint32_t e=millis(); while(millis()-e<25){ if(KL.available()){ KL.read(); break; } }   // own echo
  k81_up=true; k81_cnt=0;
  Serial.print("OK:kw1281-init sync=55 kb1="); if(kb1<16)Serial.print('0'); Serial.print(kb1,HEX);
  Serial.print(" kb2="); if(kb2<16)Serial.print('0'); Serial.println(kb2,HEX);
}

void handleLine(String line){
  line.trim();
  if (line.length()==0) return;

  String parts[6];
  int np = split(line, ':', parts, 6);
  String kw = parts[0]; kw.toUpperCase();
  set_mode(kw.c_str());                                       // title shows the current command/mode
  if (kw=="UDS"||kw=="CANX"||kw=="RAW"||kw=="SCAN") h1_cmd++; // feeds the H1 "our activity" VU bar

  if (kw=="PING"){ Serial.println("PONG"); return; }
  if (kw=="REBOOT"){   // jump to the HalfKay bootloader so the host can flash (Console "Update firmware")
    Serial.println("OK:reboot-to-bootloader"); Serial.flush(); delay(40);
    _reboot_Teensyduino_();
    return;
  }
  if (kw=="KWP"){      // Head 3 — K-line / KWP2000 for pre-CAN VAG (Serial2 + K-line transceiver)
    if (kl_ext_owned){ Serial.println("ERR:kline-busy (raw passthrough active on the 2nd USB port — close that app to use smart KWP)"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("off")){ KL.end(); kl_up=false; Serial.println("OK:kwp-off"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("baud")){   // 10400 default; 9600 for M232/AAN
      if (np<3){ Serial.println("ERR:format (KWP:baud:<n>)"); return; }
      kl_baud = strtoul(parts[2].c_str(),nullptr,10);
      Serial.print("OK:kwp-baud="); Serial.println(kl_baud); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("passthrough")){   // transparent dumb-KKL-cable mode (NefMoto etc.)
      if (np>=3) kl_baud = strtoul(parts[2].c_str(),nullptr,10);   // K-line baud (10400 default; KWP:passthrough:9600)
      KL.end(); KL.begin(kl_baud); while(KL.available()) KL.read();
      kl_passthru = true;
      Serial.print("OK:kwp-passthrough baud="); Serial.print(kl_baud);
      Serial.println(" (raw K-line<->USB; host drives the protocol; reset board to exit)"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("fast")){
      kl_fastinit(np>=3 ? (uint8_t)strtoul(parts[2].c_str(),nullptr,16) : 0x01); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("slow")){
      if (np<3){ Serial.println("ERR:format (KWP:slow:<addr>)"); return; }
      kl_slowinit((uint8_t)strtoul(parts[2].c_str(),nullptr,16)); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("raw")){
      if (np<3){ Serial.println("ERR:format (KWP:raw:<hex>)"); return; }
      uint8_t d[264]; int n=hexBytes(parts[2],d,sizeof(d)); if(n<0){ Serial.println("ERR:hex"); return; }
      kl_send(d,n); uint8_t r[264]; int rn=kl_recv(r,264,500); kl_dump("OK:",r,rn); return; }
    if (np>=2){   // KWP:<hex> — framed request -> raw response
      uint8_t d[264]; int n=hexBytes(parts[1],d,sizeof(d)); if(n<0){ Serial.println("ERR:hex"); return; }
      kl_kwp(d,n); return; }
    Serial.println("ERR:format (KWP:fast[:tgt] | KWP:slow:<addr> | KWP:passthrough[:baud] | KWP:<hex> | KWP:raw:<hex> | KWP:off)");
    return;
  }
  if (kw=="K81"){     // Head 3 — KW1281 block protocol for older pre-CAN VAG. BENCH-UNTESTED.
    if (kl_ext_owned){ Serial.println("ERR:kline-busy (raw passthrough active on the 2nd USB port)"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("off")){ KL.end(); k81_up=false; Serial.println("OK:k81-off"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("init")){
      if (np<3){ Serial.println("ERR:format (K81:init:<addr>)"); return; }
      k81_init((uint8_t)strtoul(parts[2].c_str(),nullptr,16)); return; }
    if (!k81_up){ Serial.println("ERR:k81-not-initialized (K81:init:<addr> first)"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("read")){   // read the next ECU block
      uint8_t d[64], ttl=0; int n=k81_rx(d,64,&ttl);
      if(n<0){ Serial.println("ERR:no-block"); return; } k81_dump("OK:",ttl,d,n); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("ack")){    // send ACK block 0x09, read reply
      if(!k81_tx(0x09,nullptr,0)){ Serial.println("ERR:tx-ack"); return; }
      uint8_t d[64], ttl=0; int n=k81_rx(d,64,&ttl);
      if(n<0){ Serial.println("ERR:no-block"); return; } k81_dump("OK:",ttl,d,n); return; }
    if (np>=3 && parts[1].equalsIgnoreCase("block")){  // K81:block:<title>[:<datahex>] -> send + read reply
      uint8_t title=(uint8_t)strtoul(parts[2].c_str(),nullptr,16);
      uint8_t d[64]; int dn=0;
      if(np>=4){ dn=hexBytes(parts[3],d,sizeof(d)); if(dn<0){ Serial.println("ERR:hex"); return; } }
      if(!k81_tx(title,d,dn)){ Serial.println("ERR:tx-block"); return; }
      uint8_t r[64], ttl=0; int n=k81_rx(r,64,&ttl);
      if(n<0){ Serial.println("ERR:no-block"); return; } k81_dump("OK:",ttl,r,n); return; }
    Serial.println("ERR:format (K81:init:<addr> | K81:read | K81:ack | K81:block:<title>[:<hex>] | K81:off)");
    return;
  }
  if (kw=="SLCAN"){ slcan_mode=true; Serial.println("OK:slcan (Lawicel mode on Head 1; reset board to exit)"); return; }
  if (kw=="INFO"){
    Serial.print("CERBERUS:"); Serial.print(CERBERUS_VERSION);
#if defined(ARDUINO_TEENSY41)
    Serial.print(" board=T4.1 product=Cerberus");   // 4.1 flagship: KWP + CAN + DoIP
#elif defined(ARDUINO_TEENSY40)
    Serial.print(" board=T4.0 product=Orthrus");     // 4.0 sibling: KWP + CAN (no DoIP — no Ethernet)
#else
    Serial.print(" board=T4.x product=Cerberus");
#endif
    Serial.print(" CAN1="); Serial.print(bus1_baud);
    Serial.print(" CAN2="); Serial.print(bus2_baud);
    Serial.print(" tmo="); Serial.print(UDS_TIMEOUT_MS);
    Serial.print(" respmax=4096"); Serial.print(" monring="); Serial.print(MON_RING);
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    Serial.print(" kline2=raw");      // 2nd USB CDC port is an always-dumb K-line KKL cable
#endif
    Serial.println();
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
  if (kw=="EMU"){
    // EMU:on:bus:REQ:RESP | EMU:add:PREFIXHEX:RESPHEX | EMU:clear | EMU:off | EMU:stat
    // Emulate a module on Head 1: answer UDS on REQ id from RESP id by request-prefix rules.
    if (np>=2 && parts[1].equalsIgnoreCase("off")){ emu_on=false; Serial.println("OK:emu-off"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("clear")){ emu_nrules=0; Serial.println("OK:emu-clear"); return; }
    if (np>=2 && parts[1].equalsIgnoreCase("stat")){
      Serial.print("EMUSTAT:on="); Serial.print(emu_on?1:0);
      Serial.print(" req="); Serial.print(emu_req,HEX);
      Serial.print(" resp="); Serial.print(emu_resp,HEX);
      Serial.print(" rules="); Serial.print(emu_nrules);
      Serial.print(" hits="); Serial.println(emu_hits);
      return;
    }
    if (np>=4 && parts[1].equalsIgnoreCase("add")){
      if (emu_nrules>=EMU_RULES){ Serial.println("ERR:emu-rules-full"); return; }
      EmuRule& rl = emu_rules[emu_nrules];
      int pl  = hexBytes(parts[2], rl.prefix, sizeof(rl.prefix));
      int rln = hexBytes(parts[3], rl.resp,   sizeof(rl.resp));
      if (pl<=0 || rln<=0){ Serial.println("ERR:hex (prefix<=8B / resp<=64B)"); return; }
      rl.plen=pl; rl.rlen=rln; emu_nrules++;
      Serial.print("OK:emu-rule "); Serial.println(emu_nrules);
      return;
    }
    if (np>=5 && parts[1].equalsIgnoreCase("on")){
      emu_req  = strtoul(parts[3].c_str(),nullptr,16);
      emu_resp = strtoul(parts[4].c_str(),nullptr,16);
      emu_hits = 0; emu_on = true;
      Serial.print("OK:emu-on req="); Serial.print(emu_req,HEX);
      Serial.print(" resp="); Serial.println(emu_resp,HEX);
      return;
    }
    Serial.println("ERR:format (EMU:on:bus:REQ:RESP | EMU:add:PREFIX:RESP | EMU:clear|off|stat)");
    return;
  }
  if (kw=="BAUD"){
    // BAUD                 -> report both rates
    // BAUD:<bus>:<rate>    -> retune one head NOW, no reflash
    //
    // Why: a head at the wrong bitrate sees a perfectly silent bus and raises NO error
    // flags -- symptom-identical to an unwired tap. That ambiguity is the whole reason
    // hunting a non-diagnostic bus (comfort off the gateway A5/A15, etc.) is painful.
    // Retune and re-SNIFF instead of rebuilding per guess; STATS:<bus> then tells the two
    // apart -- a wrong rate shows crc/frm/stf errors, a dead tap shows nothing at all.
    if (np < 3){
      Serial.print("BAUD:1="); Serial.print(bus1_baud);
      Serial.print(" 2=");     Serial.println(bus2_baud);
      return;
    }
    int bus = parts[1].toInt();
    uint32_t rate = strtoul(parts[2].c_str(), nullptr, 10);
    if (bus!=1 && bus!=2){ Serial.println("ERR:bus (1|2)"); return; }
    if (rate < 10000 || rate > 1000000){ Serial.println("ERR:rate (10000-1000000)"); return; }
    // Re-init each head into ITS OWN resting mode: Head 1 is the active VCI, Head 2 is the
    // always-listen-only logger (same invariant SNIFF's restore has to respect).
    if (bus==1){ bus1_baud = rate; Head1.setBaudRate(bus1_baud);               Head1.enableFIFO(); }
    else       { bus2_baud = rate; Head2.setBaudRate(bus2_baud, LISTEN_ONLY);  Head2.enableFIFO(); }
    Serial.print("OK:baud bus="); Serial.print(bus);
    Serial.print(" rate=");       Serial.println(rate);
    return;
  }

  if (kw=="MODE"){
    // MODE:vci|log|sniff|dual — set both heads' posture in one shot (the high-level convenience over
    // HEAD2/HEAD1). vci = H1 active VCI + H2 listen-only logger (boot default); sniff = BOTH heads
    // listen-only + MON on (zero bus footprint — safe to log a live ODIS/dealer session); dual =
    // both active VCIs. `MODE` alone reports. Steady OLED title reflects it.
    if (np>=2){
      String m=parts[1]; m.toLowerCase();
      if (m=="vci"){
        Head1.setBaudRate(bus1_baud);              Head1.enableFIFO();
        Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
        mon_on=false; cur_mode_name="VCI";
      } else if (m=="sniff"){
        Head1.setBaudRate(bus1_baud, LISTEN_ONLY); Head1.enableFIFO();   // H1 silent too
        Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
        drain_stale(Head1); drain_stale(Head2);   // this path arms MON straight off a re-init
        mon_idlo=0x000; mon_idhi=0x7FF; mon_head=mon_tail=0; mon_dropped=0; mon_peak=0; mon_total=0;
        mon_t0=millis(); mon_on=true; cur_mode_name="SNIFF";
      } else if (m=="log"){
        // The everyday working mode: Head 1 is the ACTIVE VCI, Head 2 is the passive logger,
        // and MON is armed in one shot. Reachable before as MODE:vci followed by MON:on -- but
        // that ordering is a trap, because MODE:vci sets mon_on=false, so doing it the other way
        // round (or any later MODE) silently kills the logging while looking fine.
        Head1.setBaudRate(bus1_baud);              Head1.enableFIFO();
        Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
        drain_stale(Head2);                        // no phantom frames at the head of the log
        mon_idlo=0x000; mon_idhi=0x7FF; mon_head=mon_tail=0; mon_dropped=0; mon_peak=0; mon_total=0;
        mon_t0=millis(); mon_on=true; cur_mode_name="LOG";
      } else if (m=="dual"){
        Head1.setBaudRate(bus1_baud); Head1.enableFIFO();
        Head2.setBaudRate(bus2_baud); Head2.enableFIFO();
        mon_on=false; cur_mode_name="DUAL";
      } else { Serial.println("ERR:format (MODE:vci|log|sniff|dual)"); return; }
      Serial.print("OK:mode="); Serial.println(cur_mode_name);
      return;
    }
    Serial.print("MODE:"); Serial.println(cur_mode_name);
    return;
  }
  if (kw=="HEAD2"){
    // HEAD2:active -> Head 2 becomes a 2nd ACTIVE VCI (normal TX/RX, mirrors Head 1) so
    // UDS:2 / RAW:2 / CANX:2 / SCAN:2 actually transmit on it — used to com-test the Head-2
    // transceiver against the live bus. HEAD2:lom -> restore listen-only (the MON logger role).
    // MON:on also forces LOM. Default at boot is LOM.
    if (np>=2 && parts[1].equalsIgnoreCase("active")){
      mon_on=false;
      Head2.setBaudRate(bus2_baud); Head2.enableFIFO();          // normal mode = full TX+RX
      Serial.println("OK:head2-active (TX) — UDS:2/RAW:2/CANX:2/SCAN:2 now drive Head 2");
      return;
    }
    if (np>=2 && parts[1].equalsIgnoreCase("lom")){
      Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
      Serial.println("OK:head2-lom (listen-only logger)");
      return;
    }
    Serial.println("ERR:format (HEAD2:active | HEAD2:lom)");
    return;
  }
  if (kw=="SELFTEST"){
    // Factory / bring-up QC — verifies a unit with NO car needed:
    //   1) CAN1 internal loopback  (controller core)
    //   2) CAN2 internal loopback  (controller core)
    //   3) H1->H2 wire test: Head 1 transmits, Head 2 receives -> proves both transceivers + the
    //      shared CANH/CANL link + the TX/RX orientation. Head 2 runs NORMAL (not LOM) so it ACKs
    //      Head 1's frames itself -> fully self-contained, NO car/other node needed to ACK.
    bool p1=false,p2=false; uint32_t saw=0;
    CAN_message_t r;
    // 1) CAN1 core
    Head1.setBaudRate(bus1_baud); Head1.enableFIFO(); Head1.enableLoopBack(true); delay(5);
    { CAN_message_t m; m.id=0x111; m.len=8; for(int i=0;i<8;i++)m.buf[i]=0x5A; Head1.write(m);
      uint32_t t=millis(); while(millis()-t<60){ if(Head1.read(r)&&r.id==0x111){p1=true;break;} } }
    Head1.enableLoopBack(false);
    // 2) CAN2 core
    Head2.setBaudRate(bus2_baud); Head2.enableFIFO(); Head2.enableLoopBack(true); delay(5);
    { CAN_message_t m; m.id=0x222; m.len=8; for(int i=0;i<8;i++)m.buf[i]=0xA5; Head2.write(m);
      uint32_t t=millis(); while(millis()-t<60){ if(Head2.read(r)&&r.id==0x222){p2=true;break;} } }
    Head2.enableLoopBack(false);
    // 3) H1 -> H2 over the wire. Head 2 NORMAL (not LOM) so it ACKs Head 1 -> self-contained.
    Head1.setBaudRate(bus1_baud); Head1.enableFIFO();
    Head2.setBaudRate(bus2_baud); Head2.enableFIFO();
    { CAN_message_t m; m.id=0x321; m.len=8; for(int i=0;i<8;i++)m.buf[i]=0x33;
      for(int i=0;i<20;i++){ Head1.write(m);
        uint32_t t=millis(); while(millis()-t<5){ if(Head2.read(r)&&r.id==0x321) saw++; } } }
    bool pw = (saw>0);
    // restore: Head 1 active VCI, Head 2 listen-only logger
    Head1.setBaudRate(bus1_baud); Head1.enableFIFO();
    Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO(); mon_on=false;
    Serial.print("SELFTEST: can1_loop="); Serial.print(p1?"PASS":"FAIL");
    Serial.print(" can2_loop=");          Serial.print(p2?"PASS":"FAIL");
    Serial.print(" h1->h2_wire=");        Serial.print(pw?"PASS":"FAIL");
    Serial.print(" (head2_saw="); Serial.print(saw); Serial.print(")  => ");
    Serial.println((p1&&p2&&pw) ? "UNIT OK"
                 : (p1&&p2)     ? "controllers OK; wire test inconclusive (saw=0) — needs a TERMINATED bus + a node: "
                                  "connect to the vehicle, or a 120R bench jig, then re-run. If it still reads 0 on a "
                                  "good bus, check the TX/RX cross + CANH/CANL link."
                                : "CONTROLLER FAIL");
    return;
  }
  if (kw=="H2TEST"){
    // Diagnostic: does Head 2's TX *physically* reach the bus? Head 1 listens (LISTEN_ONLY) while
    // Head 2 transmits a marker frame (ID 0x100) 20x. Splits a dead Head 2 into TX-path vs RX-path:
    //   head1_saw>0  -> Head-2 TX reaches the wire (TX/power/Rs/CTX ok); chase RX (CRX->pin0).
    //   head1_saw==0 -> Head-2 TX never hits the wire (power / Rs-standby / CTX->pin1 / transceiver).
    // Needs another node on the bus to ACK (the car's gateway does). Restores both heads after.
    CAN_message_t m; m.id=0x100; m.len=8; for (int i=0;i<8;i++) m.buf[i]=0xA5;
    Head1.setBaudRate(bus1_baud, LISTEN_ONLY); Head1.enableFIFO();
    Head2.setBaudRate(bus2_baud);              Head2.enableFIFO();   // Head 2 active TX
    uint32_t seen=0, sent=0;
    for (int i=0;i<20;i++){
      if (Head2.write(m)) sent++;
      uint32_t t=millis();
      while (millis()-t < 15){ CAN_message_t r; if (Head1.read(r) && r.id==0x100) seen++; }
    }
    CAN_error_t e2; Head2.error(e2,false);
    Head1.setBaudRate(bus1_baud);              Head1.enableFIFO();   // restore active VCI
    Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();   // restore logger
    mon_on=false;
    Serial.print("H2TEST: head2_queued="); Serial.print(sent);
    Serial.print(" head1_saw="); Serial.print(seen);
    Serial.print(" head2_txerr="); Serial.println(e2.TX_ERR_COUNTER);
    Serial.println(seen>0
      ? "  => Head-2 TX REACHES the bus (TX path OK) -> RX is the break: chase CRX->pin0"
      : "  => Head-2 TX does NOT reach the bus (TX path dead): power / Rs->GND / CTX->pin1 / transceiver");
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
    if (np>=2 && parts[1].equalsIgnoreCase("port")){
      // MON:PORT -> report ; MON:PORT:1|2 -> choose the sink (see mon_port above)
      if (np < 3){ Serial.print("M2PORT:"); Serial.println(mon_port); return; }
      int p = parts[2].toInt();
      if (p != 1 && p != 2){ Serial.println("ERR:port (1|2)"); return; }
      if (p == 2 && !MON_HAS_PORT2){ Serial.println("ERR:no-2nd-cdc (single-serial build)"); return; }
      if (p == 2 && kl_passthru){
        Serial.println("ERR:kline-passthrough-active (it owns the 2nd CDC; reset to exit)"); return;
      }
      mon_port = (uint8_t)p;
      Serial.print("OK:mon-port="); Serial.print(mon_port);
      Serial.println(mon_port==2 ? " (2nd CDC; K-line bridge stands down)" : " (command port)");
      return;
    }
    if (np>=2 && parts[1].equalsIgnoreCase("stat")){
      Serial.print("M2STAT:on="); Serial.print(mon_on?1:0);
      Serial.print(" port="); Serial.print(mon_port);
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
      Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
      drain_stale(Head2);           // else the re-init's stale mailboxes open the log with
                                    // phantom "M2:0:0:" frames and inflate total= from frame 1
      mon_head=mon_tail=0; mon_dropped=0; mon_peak=0; mon_total=0;   // fresh ring per session
      mon_t0 = millis(); mon_on = true;
      Serial.println("OK:mon-on");
      return;
    }
    Serial.println("ERR:format (MON:on[:idlo:idhi] | MON:off | MON:stat | MON:PORT[:1|2])");
    return;
  }
  if (kw=="SNIFF"){
    if (np<2){ Serial.println("ERR:format (SNIFF:bus:ms[:idlo:idhi])"); return; }
    int bus = parts[1].toInt();
    uint32_t ms   = (np>=3)?(uint32_t)parts[2].toInt():0;
    uint32_t idlo = (np>=4)?strtoul(parts[3].c_str(),nullptr,16):0x000;
    uint32_t idhi = (np>=5)?strtoul(parts[4].c_str(),nullptr,16):0x7FF;
    // Passive sniff in hardware LISTEN-ONLY (LOM): never ACK, never disturb the bus
    // (safe to run alongside ODIS). Restore each head to ITS OWN resting mode on exit --
    // Head 1 is the active VCI, but Head 2 is the always-listen-only logger (boot sets it
    // that way at Head2.begin(), MODE:vci and the H2TEST restore both put it back that
    // way, and HARDWARE.md promises it "never ACKs or drives"). Restoring Head 2 to TX
    // left a SECOND actively-ACKing node on the same OBD 6/14 pair until the next reboot.
    // enableFIFO() is re-asserted after each baud/LOM re-init so reception keeps working.
    if (bus==1){
      Head1.setBaudRate(bus1_baud, LISTEN_ONLY); Head1.enableFIFO();
      do_sniff(Head1, ms, idlo, idhi);
      Head1.setBaudRate(bus1_baud, TX); Head1.enableFIFO();
    } else if (bus==2){
      Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
      do_sniff(Head2, ms, idlo, idhi);
      Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.enableFIFO();
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
  if (kw=="TP20"){
    // TP20                      -> report the open channel
    // TP20:CLOSE                -> disconnect (A8)
    // TP20:<bus>:<dest>:<HEX>   -> one KWP request/response over TP2.0
    //
    // The channel is opened on first use and KEPT OPEN (loop() keepalives it), because
    // setup costs ~2 round trips; a second request to the same dest reuses it. A request
    // to a different dest transparently closes and reopens.
    if (np < 2){
      if (tp20_bus) { Serial.print("TP20:bus="); Serial.print(tp20_bus);
                      Serial.print(" dest="); Serial.print(tp20_dest, HEX);
                      Serial.print(" rx=");   Serial.print(tp20_rxid, HEX);
                      Serial.print(" tx=");   Serial.println(tp20_txid, HEX); }
      else Serial.println("TP20:closed");
      return;
    }
    if (parts[1].equalsIgnoreCase("CLOSE")){ tp20_disconnect(); Serial.println("OK:tp20-closed"); return; }
    if (np < 4){ Serial.println("ERR:format (TP20:bus:DEST:HEX | TP20:CLOSE | TP20)"); return; }
    int bus       = parts[1].toInt();
    uint8_t dest  = (uint8_t)strtoul(parts[2].c_str(), nullptr, 16);
    if (bus != 1 && bus != 2){ Serial.println("ERR:bus (1|2)"); return; }
    static uint8_t req[520];
    int reqlen = hexBytes(parts[3], req, sizeof(req));
    if (reqlen <= 0){ Serial.println("ERR:hex (too long / odd)"); return; }

    tp20_busy = true;                                  // hold off the background keepalive
    if (tp20_bus && (tp20_dest != dest || tp20_bus != bus)) tp20_disconnect();
    if (!tp20_bus){
      // We pick the ECU->us id ourselves; 0x300|(dest&0xF) keeps channels distinct and
      // matches what the dealer tool happens to choose. The ECU names its own id in the reply.
      if (!tp20_connect(bus, dest, 0x300 | (dest & 0x0F))){
        tp20_busy = false;
        Serial.println("ERR:tp20-connect (no D0 -- module absent, asleep, or not routed)");
        return;
      }
    }
    if (!tp20_send(req, reqlen)){
      tp20_busy = false; tp20_disconnect();
      Serial.println("ERR:tp20-send (no block ack)");
      return;
    }
    static uint8_t resp[1024];
    int n = tp20_recv(resp, sizeof(resp), UDS_TIMEOUT_MS);
    tp20_ka = millis();                                // the exchange itself kept it alive
    tp20_busy = false;
    if (n < 0){ Serial.println("ERR:tp20-timeout"); return; }
    Serial.print("OK:");
    for (int i = 0; i < n; i++){ if (resp[i] < 16) Serial.print('0'); Serial.print(resp[i], HEX); }
    Serial.println();
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

  Serial.println("ERR:unknown (PING|INFO|MODE|BAUD|SCAN|SNIFF|MON|HEAD2|H2TEST|SELFTEST|REBOOT|KWP|K81|EMU|STATS|TP|TP20|RAW|CANX|UDS|SLCAN|TX:RX:HEX)");
}

void setup(){
  Serial.begin(115200);
  Head1.begin(); Head1.setBaudRate(bus1_baud); Head1.setMaxMB(16); Head1.enableFIFO();
  Head2.begin(); Head2.setBaudRate(bus2_baud, LISTEN_ONLY); Head2.setMaxMB(16); Head2.enableFIFO();
  oled_init();
}

String inbuf;
void loop(){
  if (kl_passthru){                                   // transparent K-line <-> USB byte bridge (dumb KKL
    while (KL.available())     Serial.write(KL.read());  // cable for NefMoto etc.): no framing, no echo
    while (Serial.available()) KL.write(Serial.read());  // suppression — the host owns the protocol. Reset to exit.
    return;
  }
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
    emu_service();                                    // module emulation responder (Head 1)
    tp_service();                                     // native: non-blocking TesterPresent keep-alive
    tp20_service();                                   // native: TP2.0 channel keepalive (A3 every ~1s)
    kline_bridge();                                   // 2nd USB port = always-dumb K-line KKL cable
  }
  oled_update();                                      // throttled OLED HUD (no-op if no panel)
}
