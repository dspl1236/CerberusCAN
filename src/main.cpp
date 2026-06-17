/*
 * Cerberus — Teensy 4.1 tri-CAN OBD interface for VAG
 *
 *   Head 1: Powertrain/Diagnostic CAN  (CAN1, 500 kbps, OBD 6/14)   -- active
 *   Head 2: Comfort/Convenience CAN     (CAN2, 100 kbps, OBD 3/11)   -- active (sniff + UDS)
 *   Head 3: CAN-FD spare                (CAN3, pins 30/31)           -- stub (needs FD xcvr)
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
 *   SNIFF:<bus>:<ms>               passive dump (ms=0 = until any serial byte)
 *   TP:<bus>:<TXID>:<ms>          background TesterPresent keep-alive (non-blocking) | TP:STOP to end
 *   INFO                           firmware + bus config
 *   PING                           -> PONG
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

// Passive monitor. ms=0 -> run until any serial byte arrives.
// NOTE: not hardware listen-only — Cerberus ACKs at the configured baud, so only
// sniff a bus once you know its speed. Wrong baud can inject error frames.
template <typename BUS>
static void do_sniff(BUS& bus, uint32_t ms){
  uint32_t start=millis(), count=0;
  for (;;){
    if (ms!=0 && (int32_t)((start+ms)-millis())<=0) break;
    if (ms==0 && Serial.available()){ while(Serial.available()) Serial.read(); break; }
    CAN_message_t r;
    if (bus.read(r)){
      Serial.print("RX:"); Serial.print(millis()-start); Serial.print(':');
      Serial.print(r.id, HEX); Serial.print(':');
      printHex(r.buf, r.len); Serial.println();
      count++;
    }
  }
  Serial.print("DONE:"); Serial.println(count);
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
  if (kw=="INFO"){
    Serial.print("CERBERUS:"); Serial.print(CERBERUS_VERSION);
    Serial.print(" CAN1="); Serial.print(BUS1_BAUD);
    Serial.print(" CAN2="); Serial.print(BUS2_BAUD);
    Serial.print(" tmo="); Serial.print(UDS_TIMEOUT_MS);
    Serial.print(" respmax=4096"); Serial.println();
    return;
  }
  if (kw=="SNIFF"){
    if (np<2){ Serial.println("ERR:format (SNIFF:bus:ms)"); return; }
    int bus = parts[1].toInt();
    uint32_t ms = (np>=3)?(uint32_t)parts[2].toInt():0;
    if (bus==1) do_sniff(Head1, ms);
    else if (bus==2) do_sniff(Head2, ms);
    else Serial.println("ERR:bus (1|2)");
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

  Serial.println("ERR:unknown (PING|INFO|SCAN|SNIFF|TP|RAW|UDS|TX:RX:HEX)");
}

void setup(){
  Serial.begin(115200);
  Head1.begin(); Head1.setBaudRate(BUS1_BAUD); Head1.setMaxMB(16); Head1.enableFIFO();
  Head2.begin(); Head2.setBaudRate(BUS2_BAUD); Head2.setMaxMB(16); Head2.enableFIFO();
}

String inbuf;
void loop(){
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\n' || c=='\r'){ if (inbuf.length()){ handleLine(inbuf); inbuf=""; } }
    else if (inbuf.length() < 1100) inbuf += c;       // room for a full TransferData chunk
  }
  tp_service();                                       // non-blocking TesterPresent keep-alive
}
