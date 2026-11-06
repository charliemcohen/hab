// SCI-UNO: I2C slave 0x42 for Mega frames + local GPS on the shield (SoftSerial D8/D7) -> SD CSV

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>  // DateTime math only, no RTC chip on this shield
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include "sci_link.h"

#define FW_NAME     "SCI-UNO"
#define FW_VERSION  "1.3.0-GPS-UNO"

#define SD_CS_PIN         10
#define USB_BAUD          115200
#define FLUSH_INTERVAL_MS 4000UL  // flush in the gap after a frame, SD sync can take 10-100 ms
#define STATUS_PERIOD_MS  5000UL
#define SD_RETRY_MS       10000UL

static SoftwareSerial gpsPort(8, 7);
#define GPS_NMEA_MAX 84
static char gpsLine[GPS_NMEA_MAX];
static uint8_t gpsLen = 0;
static bool gpsOverflow = false;
static uint16_t gpsLinesOK = 0, gpsLinesBad = 0;
static uint32_t gpsLastMs = 0;

File     sci;
sci_rx_t rx;

static uint16_t file_idx = 0;
static char     sci_name[13];
static volatile bool sd_ok = false;
static uint8_t  rtc_src = 0;
static uint32_t uno_seq = 0;
static uint32_t frames_ok = 0, crc_err = 0, fmt_err = 0, ovf_err = 0, seq_gaps = 0, seq_missing = 0, seq_resets = 0, sd_err = 0;
static uint32_t last_seq = 0; static bool have_seq = false;
static uint32_t last_flush_ms = 0, last_status_ms = 0, last_sd_retry_ms = 0, last_mem_ms = 0;
#define MEM_ROW_PERIOD_MS 300000UL
#define MEM_ROW_FIRST_MS  60000UL
static bool     dirty = false;
static uint8_t  mega_link_ver = 0;

static uint32_t time_base_epoch = 0, time_base_ms = 0;
static bool have_time = false;
#define UNO_I2C_ADDR 0x42
#define I2C_DATA_MAX 28
static volatile uint8_t mb_data[I2C_DATA_MAX], mb_len = 0, mb_seq = 0;
static volatile bool mb_pending = false;
static volatile uint8_t last_committed = 255;
static volatile uint16_t mb_rejected = 0;

// ISRs only copy to the mailbox and ACK. no SD, Serial, or parsing in interrupt context
static void onI2CReceive(int count) {
  if (count < 2) { while (Wire.available()) Wire.read(); mb_rejected++; return; }
  uint8_t seq = Wire.read();
  uint8_t n = Wire.read();
  if (n == 0 || n > I2C_DATA_MAX || n + 2 != count || mb_pending) {
    while (Wire.available()) Wire.read();
    if (seq != last_committed) mb_rejected++;
    return;
  }
  if (seq == last_committed) {
    while (Wire.available()) Wire.read();
    return;
  }
  for (uint8_t i = 0; i < n; i++) mb_data[i] = (uint8_t)Wire.read();
  mb_seq = seq;
  mb_len = n;
  mb_pending = true;
}
static void onI2CRequest() {
  uint8_t response[2] = {last_committed, (uint8_t)(sd_ok ? 1 : 0)};
  Wire.write(response, 2);
}

extern char *__brkval, __heap_start;
#define STACK_PAINT 0xA5
// paint free RAM with 0xA5; lowest overwritten address = worst-case stack depth (2 KB SRAM)
static void stackPaint() {
  char *p = __brkval ? __brkval : &__heap_start;
  char *sp = (char *)SP;
  while (p < sp - 16) *p++ = STACK_PAINT;
}
static uint16_t stackMinFree() {
  uint8_t *p = (uint8_t *)(__brkval ? __brkval : &__heap_start);
  uint16_t n = 0;
  while (p < (uint8_t *)SP && *p == STACK_PAINT) { n++; p++; }
  return n;
}

static char *putU(char *p, uint32_t v) { ultoa(v, p, 10); return p + strlen(p); }
static char *putN(char *p, uint32_t v, uint8_t digits) {
  for (int8_t i = digits - 1; i >= 0; i--) { p[i] = (char)('0' + v % 10); v /= 10; }
  return p + digits;
}
static char *putP(char *p, PGM_P s) { char c; while ((c = pgm_read_byte(s++))) *p++ = c; return p; }

// empty until GPS has time. never a fake date
static void rtcStamp(char *out) {
  out[0] = 0;
  if (!have_time) return;
  DateTime t(time_base_epoch + (uint32_t)((millis() - time_base_ms) / 1000UL));
  char *p = out;
  p = putN(p, t.year(), 4); *p++ = '-'; p = putN(p, t.month(), 2); *p++ = '-'; p = putN(p, t.day(), 2); *p++ = 'T';
  p = putN(p, t.hour(), 2); *p++ = ':'; p = putN(p, t.minute(), 2); *p++ = ':'; p = putN(p, t.second(), 2); *p = 0;
}

static bool sdWriteLine(const char *s, size_t n) {
  if (!sd_ok || !sci) return false;
  size_t w = sci.write((const uint8_t *)s, n);
  if (n && w != n) { sd_err++; sd_ok = false; return false; }
  dirty = true;
  return true;
}
static bool sdWriteP(PGM_P p) {
  char c; bool ok = true;
  while ((c = pgm_read_byte(p++)) && ok) ok = sdWriteLine(&c, 1);
  return ok;
}
static void sdFlush() { if (sd_ok && sci && dirty) { sci.flush(); dirty = false; last_flush_ms = millis(); } }

static void writePrefix(char rec) {
  char b[52]; char *p = b;
  *p++ = rec; *p++ = ','; rtcStamp(p); p += strlen(p); *p++ = ',';
  p = putU(p, rtc_src); *p++ = ','; p = putU(p, millis()); *p++ = ','; p = putU(p, uno_seq); *p++ = ',';
  sdWriteLine(b, (size_t)(p - b));
}
static void writeEvent(PGM_P code, const char *detail) {
  writePrefix('E'); sdWriteP(code);
  if (detail) { sdWriteLine(",", 1); sdWriteLine(detail, strlen(detail)); }
  sdWriteLine("\n", 1); uno_seq++;
}
static void __attribute__((noinline)) writeEventU(PGM_P code, uint32_t a, uint32_t b) {
  char d[24]; char *p = putU(d, a); *p++ = ','; p = putU(p, b); *p = 0;
  writeEvent(code, d);
}

static bool __attribute__((noinline)) openNewFile() {
  for (uint16_t i = 0; i < 1000; i++) {
    char *p = putP(sci_name, PSTR("SCI_")); p = putN(p, i, 3); p = putP(p, PSTR(".CSV")); *p = 0;
    if (!SD.exists(sci_name)) { file_idx = i; break; }
    if (i == 999) return false;
  }
  sci = SD.open(sci_name, FILE_WRITE);
  if (!sci) return false;
  sdWriteP(PSTR("rec,rtc_utc,rtc_src,uno_ms,uno_seq," SCI_S_BODY_HEADER "\n"));
  writePrefix('B');
  char d[40]; char *p = putP(d, PSTR(FW_NAME "-" FW_VERSION ",")); p = putU(p, SCI_LINK_VERSION); *p++ = ',';
  p = putU(p, have_time); p = putP(p, PSTR(",1\n"));
  sdWriteLine(d, (size_t)(p - d)); uno_seq++;
  sci.flush(); dirty = false;
  return true;
}
static bool __attribute__((noinline)) sdInit() {
  if (sci) sci.close();
  if (!SD.begin(SD_CS_PIN)) { sd_ok = false; return false; }
  sd_ok = true;
  if (!openNewFile()) { sd_ok = false; return false; }
  return true;
}

static uint8_t hex4(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 255;
}
static bool nmeaOK(const char *s) {
  if (*s++ != '$') return false;
  uint8_t x = 0;
  while (*s && *s != '*') x ^= (uint8_t)*s++;
  if (*s++ != '*' || !s[0] || !s[1] || s[2]) return false;
  uint8_t hi=hex4(s[0]), lo=hex4(s[1]);
  return hi < 16 && lo < 16 && x == (uint8_t)((hi << 4) | lo);
}
static bool isRMC(const char *s) { return s[0] == '$' && s[3]=='R' && s[4]=='M' && s[5]=='C' && s[6]==','; }
static bool isGGA(const char *s) { return s[0] == '$' && s[3]=='G' && s[4]=='G' && s[5]=='A' && s[6]==','; }
// UTC from checksum-verified RMC only, ~1 s latency
static void updateClockFromRMC(char *line) {
  char *utc = strchr(line, ','); if (!utc) return; utc++;
  char *endUtc = strchr(utc, ','); if (!endUtc || endUtc-utc < 6) return;
  char *field = endUtc + 1, *date = NULL;
  for (uint8_t i=0; i<7; i++) { field = strchr(field, ','); if (!field) return; field++; }
  date = field;
  char *endDate = strchr(date, ','); if (!endDate || endDate-date != 6) return;
  for (uint8_t i=0; i<6; i++) if (date[i]<'0'||date[i]>'9'||utc[i]<'0'||utc[i]>'9') return;
  char keepU = utc[6], keepD = date[6]; utc[6]=0; date[6]=0;
  uint32_t t=atol(utc), d=atol(date);
  utc[6]=keepU; date[6]=keepD;
  uint8_t hh=t/10000, mm=t/100%100, ss=t%100;
  uint8_t day=d/10000, mon=d/100%100, yy=d%100;
  uint16_t yr=(uint16_t)(2000+yy);
  if (yr<2020||yr>2099||mon<1||mon>12||day<1||day>31||hh>23||mm>59||ss>59) return;
  DateTime dt(yr,mon,day,hh,mm,ss);
  if (!dt.isValid()) return;
  time_base_epoch=dt.unixtime(); time_base_ms=millis(); have_time=true; rtc_src=3;
}
// G rows share uno_ms with S rows so GPS and science line up postflight
static void logGPS(char *line) {
  if (!nmeaOK(line)) { gpsLinesBad++; return; }
  if (!isRMC(line) && !isGGA(line)) return;
  gpsLinesOK++; gpsLastMs=millis();
  if (isRMC(line)) updateClockFromRMC(line);
  if (!sd_ok) return;
  writePrefix('G'); sdWriteLine("\"", 1);
  sdWriteLine(line, strlen(line));
  sdWriteLine("\"\n", 2); uno_seq++;
}
static void pollGPS() {
  uint8_t count=0;
  while (gpsPort.available() && count++<96) {
    char c=(char)gpsPort.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (!gpsOverflow && gpsLen>8) { gpsLine[gpsLen]=0; logGPS(gpsLine); }
      gpsLen=0; gpsOverflow=false;
    } else if (!gpsOverflow && gpsLen<GPS_NMEA_MAX-1) gpsLine[gpsLen++]=c;
    else { gpsOverflow=true; gpsLinesBad++; }
  }
}

static void __attribute__((noinline)) handleLine() {
  char type; char *body; size_t bl;
  int rc = sci_validate(rx.buf, rx.len, &type, &body, &bl);
  if (rc != SCI_OK) {
    if (rc == SCI_E_CRC) crc_err++; else fmt_err++;
    writeEventU(PSTR("BAD_FRAME"), (uint32_t)rc, rx.len);
    return;
  }
  if (type == SCI_T_SAMPLE) {
    char f[12]; uint32_t seq = 0;
    if (sci_field(body, SCI_F_SEQ, f, sizeof f) > 0) seq = strtoul(f, NULL, 10);
    if (have_seq) {
      if (seq == last_seq + 1) {   }
      else if (seq > last_seq + 1) { seq_gaps++; seq_missing += seq - last_seq - 1; writeEventU(PSTR("SEQ_GAP"), last_seq, seq); }
      else { seq_resets++; writeEventU(PSTR("SEQ_RESET"), last_seq, seq); }
    }
    last_seq = seq; have_seq = true;
    writePrefix('S'); sdWriteLine(body, bl); sdWriteLine("\n", 1); uno_seq++; frames_ok++;
  } else if (type == SCI_T_DETECT || type == SCI_T_BOOT) {
    writePrefix(type); sdWriteLine(body, bl); sdWriteLine("\n", 1); uno_seq++; frames_ok++;
    if (type == SCI_T_BOOT) {
      char f[6];
      if (sci_field(body, 2, f, sizeof f) > 0) { mega_link_ver = (uint8_t)atoi(f);
        if (mega_link_ver != SCI_LINK_VERSION) { writeEventU(PSTR("LINK_VER_MISMATCH"), mega_link_ver, SCI_LINK_VERSION);
          Serial.println(F("** Mega link version differs from Uno: column names may not match **")); } }
      have_seq = false;
      Serial.println(F("[link] Mega boot frame received"));
    }
  } else {
    fmt_err++; char d[4] = { type, 0 }; writeEvent(PSTR("BAD_TYPE"), d);
  }
}

static void __attribute__((noinline)) printStatus() {
  char ts[20]; rtcStamp(ts);
  Serial.print(F("[status] rtc=")); Serial.print(ts[0] ? ts : "INVALID"); Serial.print(F(" src=")); Serial.print(rtc_src);
  Serial.print(F(" sd=")); Serial.print(sd_ok ? sci_name : "FAIL");
  Serial.print(F(" rows=")); Serial.print(uno_seq);
  Serial.print(F(" ok=")); Serial.print(frames_ok); Serial.print(F(" crc=")); Serial.print(crc_err);
  Serial.print(F(" fmt=")); Serial.print(fmt_err); Serial.print(F(" ovf=")); Serial.print(rx.overflow_count);
  Serial.print(F(" gaps=")); Serial.print(seq_gaps); Serial.print(F("/")); Serial.print(seq_missing);
  Serial.print(F(" resets=")); Serial.print(seq_resets); Serial.print(F(" sd_err=")); Serial.print(sd_err);
  Serial.print(F(" last_seq=")); Serial.print(last_seq);
  Serial.print(F(" i2c_rejected=")); Serial.print(mb_rejected);
  Serial.print(F(" gps_lines=")); Serial.print(gpsLinesOK);
  Serial.print(F(" gps_bad=")); Serial.print(gpsLinesBad);
  Serial.print(F(" gps_age_ms=")); Serial.print(gpsLastMs ? millis()-gpsLastMs : 0UL);
  Serial.print(F(" min_stack_free=")); Serial.println(stackMinFree());
}

void setup() {
  wdt_disable();
  stackPaint();
  Serial.begin(USB_BAUD);
  Serial.println();
  Serial.print(F("=== ")); Serial.print(F(FW_NAME)); Serial.print(F(" ")); Serial.print(F(FW_VERSION));
  Serial.print(F("  link v")); Serial.println(SCI_LINK_VERSION);

  gpsPort.begin(9600);
  gpsPort.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
  gpsPort.println(F("$PMTK220,1000*1F"));
  Serial.println(F("[gps] shield on D8/D7, 9600 baud; switch SOFT SERIAL"));

  if (sdInit()) { Serial.print(F("[sd] logging to ")); Serial.println(sci_name); }
  else Serial.println(F("[sd] INIT FAILED -> will retry every 10 s; nothing is being recorded"));

  Wire.begin((uint8_t)UNO_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  sci_rx_init(&rx);
  Serial.println(F("[link] I2C slave 0x42 on SDA A4 / SCL A5"));
  last_status_ms = millis(); last_flush_ms = millis();
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();
  pollGPS();
  bool got = false;
  if (mb_pending) {
    uint8_t seq = mb_seq, n = mb_len;
    for (uint8_t i = 0; i < n; i++) {
      int r = sci_rx_feed(&rx, (char)mb_data[i]);
      if (r == 1) { handleLine(); sci_rx_reset(&rx); got = true; }
      else if (r == -1) { ovf_err++; writeEvent(PSTR("RX_OVERFLOW"), NULL); }
    }
    noInterrupts(); last_committed = seq; mb_pending = false; interrupts();
  }
  pollGPS();
  uint32_t now = millis();
  if (dirty && (got || (now - last_flush_ms) > FLUSH_INTERVAL_MS * 2) && (now - last_flush_ms) >= FLUSH_INTERVAL_MS) sdFlush();
  pollGPS();
  if (!sd_ok && (now - last_sd_retry_ms) > SD_RETRY_MS) {
    last_sd_retry_ms = now;
    if (sdInit()) { Serial.print(F("[sd] recovered, new file ")); Serial.println(sci_name); writeEvent(PSTR("SD_RECOVERED"), NULL); }
    else Serial.println(F("[sd] still failing"));
  }
  if (now - last_mem_ms >= (last_mem_ms ? MEM_ROW_PERIOD_MS : MEM_ROW_FIRST_MS)) { last_mem_ms = now; writeEventU(PSTR("MEM"), stackMinFree(), (uint16_t)((char *)SP - (__brkval ? __brkval : &__heap_start))); }
  if (now - last_status_ms >= STATUS_PERIOD_MS) { last_status_ms = now; printStatus(); }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') printStatus();
    else if (c == 'f') { sdFlush(); Serial.println(F("[sd] flushed")); }
    else if (c == 'w') { sdFlush(); Serial.println(F("WATCHDOG TEST: hanging now; board must reboot within 8 s and open a new file")); Serial.flush(); for (;;) {} }
    else if (c == 'h') Serial.println(F("cmds: s=status  f=flush  w=watchdog hang test"));
  }
}
