/*
 * Cerberus — Teensy 4.1 tri-CAN OBD interface for VAG
 *
 *   Head 1: Powertrain/Diagnostic CAN  (CAN1, 500 kbps, OBD 6/14)  -- ACTIVE
 *   Head 2: Comfort/Convenience CAN     (CAN2, 100 kbps, OBD 3/11)  -- stub (FT transceiver)
 *   Head 3: CAN-FD spare                (CAN3)                       -- stub
 *
 * MVP: a USB-serial ISO-TP/UDS bridge on Head 1. The PC sends a UDS request to a
 * module's diagnostic ID; Cerberus frames it on the 500 kbps bus and returns the
 * assembled response. J533 routes, so Head 1 reaches J533/J255/J136/J285.
 *
 * Line protocol (115200, ASCII):   TXID:RXID:REQHEX   e.g.  710:77A:2200BE
 *                                   -> OK:<resphex>    or    ERR:<reason>
 *
 * GPLv3. github.com/dspl1236/CerberusCAN
 */
#include <FlexCAN_T4.h>
#include "cerberus_config.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Head1;     // diagnostic, 500k
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Head2;  // comfort, 100k (FT xcvr) -- bring up later
// FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> Head3;// CAN-FD -- bring up later

static uint8_t nib(char c){
  if (c>='0'&&c<='9') return c-'0';
  if (c>='a'&&c<='f') return c-'a'+10;
  if (c>='A'&&c<='F') return c-'A'+10;
  return 0xFF;
}
static int hexBytes(const String& s, uint8_t* out, int maxlen){
  int n=s.length(); if (n&1) return -1; int b=0;
  for (int i=0;i<n && b<maxlen;i+=2){ uint8_t hi=nib(s[i]),lo=nib(s[i+1]); if(hi>15||lo>15)return -1; out[b++]=(hi<<4)|lo; }
  return b;
}
static void printHex(const uint8_t* d,int n){ for(int i=0;i<n;i++){ if(d[i]<16)Serial.print('0'); Serial.print(d[i],HEX);} }

// One ISO-TP UDS exchange on Head 1. Returns response length or -1 on timeout.
int isotp_uds(uint32_t tx, uint32_t rx, const uint8_t* req, uint8_t reqlen, uint8_t* resp, uint16_t maxresp){
  CAN_message_t m; m.id=tx; m.len=8; m.flags.extended=0;
  for (int i=0;i<8;i++) m.buf[i]=PAD_BYTE;
  m.buf[0] = reqlen & 0x0F;                       // Single Frame
  for (int i=0;i<reqlen && i<7;i++) m.buf[1+i]=req[i];
  Head1.write(m);

  uint32_t deadline = millis()+UDS_TIMEOUT_MS;
  uint16_t total=0, got=0; bool multi=false;
  while ((int32_t)(deadline-millis()) > 0){
    CAN_message_t r;
    if (!Head1.read(r)) continue;
    if (r.id != rx) continue;
    uint8_t pci = r.buf[0] >> 4;

    if (pci == 0){                                // Single Frame
      uint8_t len = r.buf[0] & 0x0F; if (len>maxresp) len=maxresp;
      for (int i=0;i<len;i++) resp[i]=r.buf[1+i];
      if (len==3 && resp[0]==0x7F && resp[2]==0x78){ deadline=millis()+UDS_TIMEOUT_MS; continue; } // pending
      return len;
    }
    else if (pci == 1){                           // First Frame
      total = ((r.buf[0] & 0x0F) << 8) | r.buf[1]; multi=true; got=0;
      for (int i=0;i<6 && got<total && got<maxresp;i++) resp[got++]=r.buf[2+i];
      CAN_message_t fc; fc.id=tx; fc.len=8; for(int i=0;i<8;i++) fc.buf[i]=PAD_BYTE;
      fc.buf[0]=0x30; fc.buf[1]=0x00; fc.buf[2]=0x00;   // CTS, BS=0, STmin=0
      Head1.write(fc);
      deadline = millis()+UDS_TIMEOUT_MS;
    }
    else if (pci == 2){                           // Consecutive Frame
      for (int i=0;i<7 && got<total && got<maxresp;i++) resp[got++]=r.buf[1+i];
      deadline = millis()+UDS_TIMEOUT_MS;
      if (got >= total) return got;
    }
    // pci == 3 (flow control from ECU): ignore, we're the tester
  }
  return (multi && got) ? (int)got : -1;
}

void handleLine(const String& line){
  int p1=line.indexOf(':'), p2=line.indexOf(':',p1+1);
  if (p1<0 || p2<0){ Serial.println("ERR:format (TXID:RXID:REQHEX)"); return; }
  uint32_t tx = strtoul(line.substring(0,p1).c_str(), nullptr, 16);
  uint32_t rx = strtoul(line.substring(p1+1,p2).c_str(), nullptr, 16);
  uint8_t req[16];
  int reqlen = hexBytes(line.substring(p2+1), req, sizeof(req));
  if (reqlen <= 0 || reqlen > 7){ Serial.println("ERR:reqlen (1..7 bytes)"); return; }
  uint8_t resp[64];
  int n = isotp_uds(tx, rx, req, reqlen, resp, sizeof(resp));
  if (n < 0){ Serial.println("ERR:timeout"); return; }
  Serial.print("OK:"); printHex(resp, n); Serial.println();
}

void setup(){
  Serial.begin(115200);
  Head1.begin();
  Head1.setBaudRate(BUS1_BAUD);
  Head1.setMaxMB(16);
  Head1.enableFIFO();
  // Head 2 (comfort 100k, FT transceiver) + Head 3 (CAN-FD) -- next milestone.
}

String inbuf;
void loop(){
  while (Serial.available()){
    char c = Serial.read();
    if (c=='\n' || c=='\r'){ if (inbuf.length()){ handleLine(inbuf); inbuf=""; } }
    else if (inbuf.length() < 128) inbuf += c;
  }
}
