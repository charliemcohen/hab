// SCI-MEGA: 1 Hz sensor sweep through PCA9548 mux -> CRC16 ASCII frames to Uno (I2C slave 0x42)

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GPS.h>
#include "sci_link.h"

#define FW_NAME     "SCI-MEGA"
#define FW_VERSION  "1.2.0-I2C"

#define UNO_ADDR      0x42
#define GPS_SERIAL    Serial2
#define GPS_BAUD      9600
#define USB_BAUD      115200
#define TEST_MODE_PIN 22

#define MUX_ADDR      0x70
// mux map, flight config 10/27. ch2 = bottom exterior cluster, ch3 = top-facing light sensors
#define CH_MCP1       5
#define CH_MCP2       1
#define CH_BOTTOM     2
#define CH_TOP        3
#define CH_LIS        4

#define ADDR_MCP9808 0x18
#define ADDR_TMP119  0x48
#define ADDR_VEML    0x10
#define ADDR_LTR     0x53
#define ADDR_MS_PT   0x76
#define ADDR_MS_RH   0x40
#define ADDR_AHT     0x38
#define ADDR_LIS     0x18
#define ADDR_BMP     0x77

#define SAMPLE_PERIOD_MS   1000UL
#define STATUS_PERIOD_MS   5000UL
#define REPROBE_PERIOD_MS  30000UL
#define GPS_STALE_MS       3000UL
#define AHT_CONV_MS        85
#define WIRE_TIMEOUT_US    25000UL
#define USE_WATCHDOG       1  // 4 s WDT. check old bootloaders don't hang after a WDT reset

#define MCP_T_MIN   -40.0f
#define MCP_T_MAX   125.0f
#define TMP_T_MIN   -55.0f
#define TMP_T_MAX   150.0f
#define MS_P_MIN    10.0f  // extended range, precision not guaranteed
#define MS_P_MAX    2000.0f
#define MS_T_MIN    -40.0f
#define MS_T_MAX    85.0f
#define BMP_P_MIN   300.0f  // qualified 300-1250 hPa, below this gets flagged oor
#define BMP_P_MAX   1250.0f
#define BMP_T_MIN   -40.0f
#define BMP_T_MAX   85.0f
#define AHT_T_MIN   -40.0f
#define AHT_T_MAX   85.0f
#define LIS_G_MAX   8.0f
#define VEML_SAT    65535
#define LTR_SAT     262143UL

Adafruit_MCP9808  mcp1, mcp2, mcp3;
Adafruit_VEML7700 veml;
Adafruit_LTR390   ltr;
Adafruit_MS8607   ms8607;
Adafruit_LIS3DH   lis;
Adafruit_BMP3XX   bmp;
Adafruit_GPS      GPS(&GPS_SERIAL);

struct Sample {
  uint32_t seq, t_ms;
  uint16_t valid, oor, synth;
  float t_mcp[3], t_tmp119;
  uint16_t veml_als, veml_white; float veml_lux;
  uint32_t ltr_uvs;
  float ms_p, ms_t, ms_rh, aht_t, aht_rh;
  float ax, ay, az, bmp_p, bmp_t;
  uint8_t gps_fix, gps_qual, gps_sats;
  float gps_lat, gps_lon, gps_alt, gps_hdop;
  uint32_t gps_utc, gps_date, gps_age;
};
static Sample S;
static uint32_t g_seq = 0;
static uint16_t det_mask = 0;
static uint16_t ack_mask = 0;
static bool     mux_ok = false;
static uint32_t i2c_timeouts = 0, mux_fail = 0, read_fail = 0;
static uint8_t  sweep_step = 0;
static bool     sweep_active = false;
static uint32_t sweep_t0 = 0, next_sample_ms = 0, next_status_ms = 0, next_reprobe_ms = 0;
static uint32_t aht_trig_ms = 0;
static bool     test_mode_cmd = false;
static uint8_t  reset_cause = 0;

static uint32_t gps_last_parse_ms = 0, gps_sentences = 0, gps_parse_fail = 0;

static char fb[SCI_LINE_BUF];
static uint32_t frames_sent = 0, frames_drop = 0, chunks_sent = 0;
static uint32_t ack_count = 0, ack_last_ms = 0, i2c_link_fail = 0;
static uint8_t i2c_chunk_seq = 0, uno_sd_ok = 0;

static bool wireTimedOut() {
  if (Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); i2c_timeouts++; return true; }
  return false;
}
static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t rc = Wire.endTransmission();
  wireTimedOut();
  return rc == 0;
}
static bool muxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)(1 << ch));
  uint8_t rc = Wire.endTransmission();
  if (wireTimedOut() || rc != 0) { mux_fail++; mux_ok = false; return false; }
  mux_ok = true;
  return true;
}
static void muxNone() { Wire.beginTransmission(MUX_ADDR); Wire.write((uint8_t)0); Wire.endTransmission(); wireTimedOut(); }

static bool tmp119Begin(uint16_t *id_out) {
  Wire.beginTransmission(ADDR_TMP119); Wire.write(0x0F);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADDR_TMP119, (uint8_t)2) != 2) return false;
  uint16_t id = ((uint16_t)Wire.read() << 8) | Wire.read();
  *id_out = id;
  if (wireTimedOut()) return false;
  if ((id & 0x0FFF) != 0x117) return false;
  Wire.beginTransmission(ADDR_TMP119); Wire.write(0x01); Wire.write(0x02); Wire.write(0x20);
  return Wire.endTransmission() == 0 && !wireTimedOut();
}
static bool tmp119Read(float *t) {
  Wire.beginTransmission(ADDR_TMP119); Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADDR_TMP119, (uint8_t)2) != 2) return false;
  int16_t raw = (int16_t)(((uint16_t)Wire.read() << 8) | Wire.read());
  if (wireTimedOut()) return false;
  if (raw == (int16_t)0x8000) return false;
  *t = raw * 0.0078125f;
  return true;
}

static bool ahtBegin() {
  if (!i2cPing(ADDR_AHT)) return false;
  Wire.beginTransmission(ADDR_AHT);
  if (Wire.requestFrom((uint8_t)ADDR_AHT, (uint8_t)1) != 1) return false;
  uint8_t st = Wire.read();
  if (!(st & 0x08)) {
    Wire.beginTransmission(ADDR_AHT); Wire.write(0xBE); Wire.write(0x08); Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    delay(10);
  }
  return !wireTimedOut();
}
static bool ahtTrigger() {
  Wire.beginTransmission(ADDR_AHT); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00);
  uint8_t rc = Wire.endTransmission();
  return rc == 0 && !wireTimedOut();
}
static uint8_t ahtCrc8(const uint8_t *d, uint8_t n) {
  uint8_t crc = 0xFF;
  while (n--) { crc ^= *d++; for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1); }
  return crc;
}
static bool ahtRead(float *t, float *rh) {
  if (Wire.requestFrom((uint8_t)ADDR_AHT, (uint8_t)7) != 7) return false;
  uint8_t b[7]; for (uint8_t i = 0; i < 7; i++) b[i] = Wire.read();
  if (wireTimedOut()) return false;
  if (b[0] & 0x80) return false;
  if (ahtCrc8(b, 6) != b[6]) return false;
  uint32_t h = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
  uint32_t tt = (((uint32_t)b[3] & 0x0F) << 16) | ((uint32_t)b[4] << 8) | b[5];
  *rh = h * 100.0f / 1048576.0f;
  *t  = tt * 200.0f / 1048576.0f - 50.0f;
  return true;
}

static const char n0[] PROGMEM = "CH5 MCP9808#1 0x18 interior air";
static const char n1[] PROGMEM = "CH1 MCP9808#2 0x18 battery area";
static const char n2[] PROGMEM = "CH2 MCP9808#3 0x18 bottom ext";
static const char n3[] PROGMEM = "CH2 TMP119    0x48 bottom ext";
static const char n4[] PROGMEM = "CH3 VEML7700  0x10 top";
static const char n5[] PROGMEM = "CH3 LTR390    0x53 top";
static const char n6[] PROGMEM = "CH2 MS8607-PT 0x76 bottom ext";
static const char n7[] PROGMEM = "CH2 MS8607-RH 0x40 bottom ext";
static const char n8[] PROGMEM = "CH2 AHT20     0x38 bottom ext";
static const char n9[] PROGMEM = "CH4 LIS3DH    0x18 interior deck";
static const char n10[] PROGMEM = "CH2 BMP390    0x77 bottom ext";
static const char n11[] PROGMEM = "GPS Serial2  (UART)";
static const char n12[] PROGMEM = "INA219 (not installed, not probed)";
static const char n13[] PROGMEM = "PCA9548A mux  0x70";
static const char *const sensorName[SB_COUNT] PROGMEM = {n0,n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13};

static uint16_t tmp119_id = 0;

static bool initSensor(uint8_t sb) {
  bool ok = false;
  switch (sb) {
    case SB_MCP1: if (muxSelect(CH_MCP1)) ok = mcp1.begin(ADDR_MCP9808, &Wire); if (ok) mcp1.setResolution(3); break;
    case SB_MCP2: if (muxSelect(CH_MCP2)) ok = mcp2.begin(ADDR_MCP9808, &Wire); if (ok) mcp2.setResolution(3); break;
    case SB_MCP3: if (muxSelect(CH_BOTTOM)) ok = mcp3.begin(ADDR_MCP9808, &Wire); if (ok) mcp3.setResolution(3); break;
    case SB_TMP119: if (muxSelect(CH_BOTTOM)) ok = tmp119Begin(&tmp119_id); break;
    case SB_VEML:
      if (muxSelect(CH_TOP)) ok = veml.begin(&Wire);
      if (ok) { veml.setGain(VEML7700_GAIN_1_8); veml.setIntegrationTime(VEML7700_IT_25MS); veml.enable(true); }
      break;
    case SB_LTR:
      if (muxSelect(CH_TOP)) ok = ltr.begin(&Wire);
      if (ok) { ltr.setMode(LTR390_MODE_UVS); ltr.setGain(LTR390_GAIN_3); ltr.setResolution(LTR390_RESOLUTION_18BIT); ltr.enable(true); }
      break;
    case SB_MS_PT:
    case SB_MS_RH:
      if (muxSelect(CH_BOTTOM)) ok = ms8607.begin(&Wire);
      if (ok) { ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096); ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_12b); }
      break;
    case SB_AHT: if (muxSelect(CH_BOTTOM)) ok = ahtBegin(); break;
    case SB_LIS:
      if (muxSelect(CH_LIS)) ok = lis.begin(ADDR_LIS);
      if (ok) { lis.setRange(LIS3DH_RANGE_8_G); lis.setDataRate(LIS3DH_DATARATE_50_HZ); }
      break;
    case SB_BMP:
      if (muxSelect(CH_BOTTOM)) ok = bmp.begin_I2C(ADDR_BMP, &Wire);
      if (ok) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      }
      break;
    case SB_MUX: ok = i2cPing(MUX_ADDR); break;
    default: break;
  }
  wireTimedOut();
  return ok;
}

static void probeAck() {
  ack_mask = 0;
  if (i2cPing(MUX_ADDR)) ack_mask |= (1 << SB_MUX); else return;
  if (muxSelect(CH_MCP1) && i2cPing(ADDR_MCP9808)) ack_mask |= 1 << SB_MCP1;
  if (muxSelect(CH_MCP2) && i2cPing(ADDR_MCP9808)) ack_mask |= 1 << SB_MCP2;
  if (muxSelect(CH_BOTTOM)) {
    if (i2cPing(ADDR_MCP9808)) ack_mask |= 1 << SB_MCP3;
    if (i2cPing(ADDR_TMP119)) ack_mask |= 1 << SB_TMP119;
    if (i2cPing(ADDR_MS_PT))  ack_mask |= 1 << SB_MS_PT;
    if (i2cPing(ADDR_MS_RH))  ack_mask |= 1 << SB_MS_RH;
    if (i2cPing(ADDR_AHT))    ack_mask |= 1 << SB_AHT;
    if (i2cPing(ADDR_BMP))    ack_mask |= 1 << SB_BMP;
  }
  if (muxSelect(CH_TOP)) {
    if (i2cPing(ADDR_VEML)) ack_mask |= 1 << SB_VEML;
    if (i2cPing(ADDR_LTR))  ack_mask |= 1 << SB_LTR;
  }
  if (muxSelect(CH_LIS) && i2cPing(ADDR_LIS)) ack_mask |= 1 << SB_LIS;
  muxNone();
}

static void detectAll(bool only_failed) {
  if (!only_failed) { det_mask = 0; probeAck(); }
  for (uint8_t sb = 0; sb < SB_COUNT; sb++) {
    if (sb == SB_GPS || sb == SB_INA219) continue;
    if (only_failed && (det_mask & (1 << sb))) continue;
    if (initSensor(sb)) det_mask |= (1 << sb); else det_mask &= ~(1 << sb);
  }
  muxNone();
}

static void printDetectTable() {
  Serial.println(F("--- sensor detection (ACK = device answered on I2C, INIT = library init) ---"));
  for (uint8_t sb = 0; sb < SB_COUNT; sb++) {
    if (sb == SB_GPS || sb == SB_INA219) continue;
    char name[36]; strcpy_P(name, (PGM_P)pgm_read_ptr(&sensorName[sb]));
    Serial.print(F("  ")); Serial.print(name);
    Serial.print(F("  ACK:")); Serial.print((ack_mask >> sb) & 1 ? F("yes") : F("NO "));
    Serial.print(F("  INIT:")); Serial.print((det_mask >> sb) & 1 ? F("PASS") : F("FAIL"));
    if (sb == SB_TMP119 && ack_mask & (1 << SB_TMP119)) { Serial.print(F("  id=0x")); Serial.print(tmp119_id, HEX); }
    Serial.println();
  }
  Serial.print(F("  GPS: ")); Serial.println(gps_sentences ? F("sentences received") : F("no NMEA received yet"));
  Serial.print(F("  det_mask=0x")); Serial.println(det_mask, HEX);
}

static void gpsPump() {
  uint8_t n = 0;
  while (GPS_SERIAL.available() && n < 96) {
    GPS.read(); n++;
    if (GPS.newNMEAreceived()) {
      if (GPS.parse(GPS.lastNMEA())) { gps_last_parse_ms = millis(); gps_sentences++; }
      else gps_parse_fail++;
    }
  }
}
static bool gpsAlive() { return gps_sentences && (millis() - gps_last_parse_ms) < GPS_STALE_MS; }

static void linkPump() {}
// stop-and-wait: 28 B chunks, Uno ACKs each one, CRC16 over the reassembled frame
static void linkSend(uint16_t len) {
  if (!len) return;
  if (len > SCI_FRAME_MAX) { frames_drop++; return; }
  uint16_t pos = 0;
  const uint32_t overall_start = millis();
  while (pos < len && millis() - overall_start < 650UL) {
    const uint8_t amount = (uint8_t)min((uint16_t)28, (uint16_t)(len - pos));
    const uint8_t seq = i2c_chunk_seq;
    bool committed = false;
    for (uint8_t attempt = 0; attempt < 4 && !committed; attempt++) {
      if (millis() - overall_start >= 650UL) break;
      Wire.beginTransmission((uint8_t)UNO_ADDR);
      Wire.write(seq);
      Wire.write(amount);
      Wire.write((const uint8_t *)(fb + pos), amount);
      uint8_t rc = Wire.endTransmission();
      if (wireTimedOut() || rc != 0) { i2c_link_fail++; gpsPump(); delay(3); continue; }
      uint32_t sent_ms = millis();
      while (millis() - sent_ms < 80UL && millis() - overall_start < 650UL) {
        gpsPump();
        if (Wire.requestFrom((uint8_t)UNO_ADDR, (uint8_t)2) == 2) {
          uint8_t ack_seq = Wire.read();
          uno_sd_ok = Wire.read();
          if (ack_seq == seq) { committed = true; ack_count++; ack_last_ms = millis(); break; }
        } else { while (Wire.available()) Wire.read(); }
        wireTimedOut();
        delay(2);
      }
      if (!committed) i2c_link_fail++;
    }
    if (!committed) { frames_drop++; return; }
    pos += amount;
    chunks_sent++;
    i2c_chunk_seq++;
  }
  if (pos == len) frames_sent++;
  else frames_drop++;
}

static uint16_t fb_len; static bool fb_trunc; static uint32_t frames_trunc = 0;
static void fbStart(char type) { fb_len = 0; fb_trunc = false; fb[fb_len++] = '$'; fb[fb_len++] = type; fb[fb_len++] = ','; }
static void fbRaw(const char *s) { while (*s) { if (fb_len >= SCI_FRAME_MAX - 7) { fb_trunc = true; return; } fb[fb_len++] = *s++; } }
static void fbSep() { if (fb_len >= SCI_FRAME_MAX - 7) { fb_trunc = true; return; } fb[fb_len++] = ','; }
static void fbU32(uint32_t v, bool sep = true) { char t[12]; ultoa(v, t, 10); fbRaw(t); if (sep) fbSep(); }
static void fbHex(uint16_t v, bool sep = true) { char t[6]; utoa(v, t, 16); for (char *p = t; *p; p++) *p = toupper(*p); fbRaw(t); if (sep) fbSep(); }
static void fbF(float v, uint8_t dp, bool valid, bool sep = true) {  // invalid -> empty field, never 0
  if (valid && !isnan(v) && !isinf(v)) { char t[16]; dtostrf(v, 1, dp, t); fbRaw(t); }
  if (sep) fbSep();
}
static uint16_t fbSeal() { if (fb_trunc) { frames_trunc++; return 0; } fb[fb_len] = 0; return (uint16_t)sci_seal(fb, fb_len, SCI_LINE_BUF); }  // truncated frame is worse than no frame

static void sendBootFrame() {
  fbStart(SCI_T_BOOT);
  fbRaw(FW_NAME); fbSep(); fbRaw(FW_VERSION); fbSep(); fbU32(SCI_LINK_VERSION); fbHex(reset_cause); fbU32(millis(), false);
  linkSend(fbSeal());
}
static void sendDetectFrame() {
  fbStart(SCI_T_DETECT);
  fbU32(millis()); fbHex(det_mask); fbHex(ack_mask); fbHex(tmp119_id); fbU32(i2c_timeouts, false);
  linkSend(fbSeal());
}

static inline bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }
static void setValid(uint8_t sb, bool ok) { if (ok) S.valid |= (1 << sb); else { S.valid &= ~(1 << sb); if (det_mask & (1 << sb)) read_fail++; } }
static void setOor(uint8_t sb, bool oor) { if (oor) S.oor |= (1 << sb); }

static bool readMcp(Adafruit_MCP9808 &m, uint8_t ch, float *t) {
  if (!muxSelect(ch) || !i2cPing(ADDR_MCP9808)) return false;  // lib returns 0.00 on NACK, so ping first
  float v = m.readTempC();
  if (wireTimedOut() || isnan(v)) return false;
  *t = v; return true;
}

// one sensor per loop() pass so the Serial2 GPS buffer never overflows
static bool sweepStep() {
  switch (sweep_step) {
    case 0:
      memset(&S, 0, sizeof S); S.seq = g_seq; S.t_ms = sweep_t0; S.valid = 0; S.oor = 0; S.synth = 0;
      break;
    case 1: if (det_mask & (1 << SB_MCP1)) { bool ok = readMcp(mcp1, CH_MCP1, &S.t_mcp[0]); setValid(SB_MCP1, ok); if (ok) setOor(SB_MCP1, !inRange(S.t_mcp[0], MCP_T_MIN, MCP_T_MAX)); } break;
    case 2: if (det_mask & (1 << SB_MCP2)) { bool ok = readMcp(mcp2, CH_MCP2, &S.t_mcp[1]); setValid(SB_MCP2, ok); if (ok) setOor(SB_MCP2, !inRange(S.t_mcp[1], MCP_T_MIN, MCP_T_MAX)); } break;
    case 3: if (det_mask & (1 << SB_MCP3)) { bool ok = readMcp(mcp3, CH_BOTTOM, &S.t_mcp[2]); setValid(SB_MCP3, ok); if (ok) setOor(SB_MCP3, !inRange(S.t_mcp[2], MCP_T_MIN, MCP_T_MAX)); } break;
    case 4: if (det_mask & (1 << SB_TMP119)) { bool ok = muxSelect(CH_BOTTOM) && tmp119Read(&S.t_tmp119); setValid(SB_TMP119, ok); if (ok) setOor(SB_TMP119, !inRange(S.t_tmp119, TMP_T_MIN, TMP_T_MAX)); } break;
    case 5: if (det_mask & (1 << SB_VEML)) {
        bool ok = muxSelect(CH_TOP) && i2cPing(ADDR_VEML);
        if (ok) { S.veml_als = veml.readALS(); S.veml_white = veml.readWhite(); S.veml_lux = veml.readLux(VEML_LUX_NORMAL); ok = !wireTimedOut(); }
        setValid(SB_VEML, ok); if (ok) setOor(SB_VEML, S.veml_als >= VEML_SAT || S.veml_white >= VEML_SAT);
      } break;
    case 6: if (det_mask & (1 << SB_LTR)) {
        bool ok = muxSelect(CH_TOP) && i2cPing(ADDR_LTR);
        if (ok) { S.ltr_uvs = ltr.readUVS(); ok = !wireTimedOut() && S.ltr_uvs <= 0xFFFFF; }
        setValid(SB_LTR, ok); if (ok) setOor(SB_LTR, S.ltr_uvs >= LTR_SAT);
      } break;
    case 7:
      if (det_mask & (1 << SB_AHT)) { if (muxSelect(CH_BOTTOM) && ahtTrigger()) aht_trig_ms = millis(); else aht_trig_ms = 0; }
      break;
    case 8: if (det_mask & (1 << SB_MS_PT)) {
        bool ok = muxSelect(CH_BOTTOM) && i2cPing(ADDR_MS_PT);
        bool ok_rh = ok && i2cPing(ADDR_MS_RH);
        if (ok) {
          sensors_event_t p, t, h;
          ms8607.getEvent(&p, &t, ok_rh ? &h : NULL);
          ok = !wireTimedOut();
          S.ms_p = p.pressure; S.ms_t = t.temperature; S.ms_rh = ok_rh ? h.relative_humidity : NAN;
          // lib never reports I2C failure -> reject non-physical values
          if (!inRange(S.ms_p, 0.0f, 5000.0f) || !inRange(S.ms_t, -60.0f, 120.0f) || isnan(S.ms_p)) ok = false;
          if (ok_rh && !inRange(S.ms_rh, -10.0f, 120.0f)) ok_rh = false;
        }
        setValid(SB_MS_PT, ok); setValid(SB_MS_RH, ok && ok_rh);
        if (ok) setOor(SB_MS_PT, !inRange(S.ms_p, MS_P_MIN, MS_P_MAX) || !inRange(S.ms_t, MS_T_MIN, MS_T_MAX));
        if (ok && ok_rh) setOor(SB_MS_RH, !inRange(S.ms_rh, 0.0f, 100.0f));
      }
      break;
    case 9: if (det_mask & (1 << SB_LIS)) {
        bool ok = muxSelect(CH_LIS) && i2cPing(ADDR_LIS);
        if (ok) { lis.read(); ok = !wireTimedOut(); S.ax = lis.x_g; S.ay = lis.y_g; S.az = lis.z_g; }
        setValid(SB_LIS, ok);
        if (ok) setOor(SB_LIS, fabs(S.ax) >= LIS_G_MAX || fabs(S.ay) >= LIS_G_MAX || fabs(S.az) >= LIS_G_MAX);
      } break;
    case 10: if (det_mask & (1 << SB_BMP)) {
        bool ok = muxSelect(CH_BOTTOM) && bmp.performReading();
        if (ok) { ok = !wireTimedOut(); S.bmp_p = bmp.pressure / 100.0f; S.bmp_t = bmp.temperature; }
        setValid(SB_BMP, ok);
        if (ok) setOor(SB_BMP, !inRange(S.bmp_p, BMP_P_MIN, BMP_P_MAX) || !inRange(S.bmp_t, BMP_T_MIN, BMP_T_MAX));
      } break;
    case 11:
      if (det_mask & (1 << SB_AHT)) {
        if (aht_trig_ms && (millis() - aht_trig_ms) < AHT_CONV_MS) return false;
        bool ok = aht_trig_ms && muxSelect(CH_BOTTOM) && ahtRead(&S.aht_t, &S.aht_rh);
        setValid(SB_AHT, ok); if (ok) setOor(SB_AHT, !inRange(S.aht_t, AHT_T_MIN, AHT_T_MAX) || !inRange(S.aht_rh, 0.0f, 100.0f));
      }
      muxNone();
      break;
    case 12: {
      bool alive = gpsAlive();
      if (alive) S.valid |= (1 << SB_GPS);
      S.gps_fix = GPS.fix ? 1 : 0; S.gps_qual = GPS.fixquality; S.gps_sats = GPS.satellites;
      S.gps_lat = GPS.latitudeDegrees; S.gps_lon = GPS.longitudeDegrees; S.gps_alt = GPS.altitude; S.gps_hdop = GPS.HDOP;
      S.gps_utc = (uint32_t)GPS.hour * 10000UL + (uint32_t)GPS.minute * 100UL + GPS.seconds;
      S.gps_date = (uint32_t)(2000 + GPS.year) * 10000UL + (uint32_t)GPS.month * 100UL + GPS.day;
      S.gps_age = gps_sentences ? (millis() - gps_last_parse_ms) : 999999UL;
      if (!alive) S.gps_fix = 0;
      break; }
    default: return true;
  }
  sweep_step++;
  return sweep_step > 12;
}

static bool testModeActive() { return test_mode_cmd || digitalRead(TEST_MODE_PIN) == LOW; }

// bench test mode only (pin 22 low): fills dead sensors, flagged in synth
static void applyTestData() {
  float k = (float)(S.seq % 60);
  auto syn = [&](uint8_t sb) { return !(S.valid & (1 << sb)); };
  if (syn(SB_MCP1))   { S.t_mcp[0] = 20.0f + k * 0.1f; S.synth |= 1 << SB_MCP1; }
  if (syn(SB_MCP2))   { S.t_mcp[1] = 21.0f + k * 0.1f; S.synth |= 1 << SB_MCP2; }
  if (syn(SB_MCP3))   { S.t_mcp[2] = 22.0f + k * 0.1f; S.synth |= 1 << SB_MCP3; }
  if (syn(SB_TMP119)) { S.t_tmp119 = 23.0f + k * 0.01f; S.synth |= 1 << SB_TMP119; }
  if (syn(SB_VEML))   { S.veml_als = 1000 + (uint16_t)k; S.veml_white = 1200 + (uint16_t)k; S.veml_lux = 1843.2f + k; S.synth |= 1 << SB_VEML; }
  if (syn(SB_LTR))    { S.ltr_uvs = 500 + (uint32_t)k; S.synth |= 1 << SB_LTR; }
  if (syn(SB_MS_PT))  { S.ms_p = 1013.25f - k; S.ms_t = 24.0f; S.synth |= 1 << SB_MS_PT; }
  if (syn(SB_MS_RH))  { S.ms_rh = 40.0f + k * 0.5f; S.synth |= 1 << SB_MS_RH; }
  if (syn(SB_AHT))    { S.aht_t = 24.5f; S.aht_rh = 41.0f + k * 0.5f; S.synth |= 1 << SB_AHT; }
  if (syn(SB_LIS))    { S.ax = 0.01f; S.ay = -0.02f; S.az = 1.00f; S.synth |= 1 << SB_LIS; }
  if (syn(SB_BMP))    { S.bmp_p = 1012.80f - k; S.bmp_t = 24.2f; S.synth |= 1 << SB_BMP; }
  if (!S.gps_fix) {
    S.gps_fix = 1; S.gps_qual = 1; S.gps_sats = 8;
    S.gps_lat = 29.760400f; S.gps_lon = -95.369800f; S.gps_alt = 30.0f + k; S.gps_hdop = 1.5f;
    uint32_t s = (millis() / 1000UL) % 86400UL;
    S.gps_utc = (s / 3600) * 10000UL + ((s / 60) % 60) * 100UL + (s % 60);
    S.gps_date = 20000101UL; S.gps_age = 0; S.synth |= 1 << SB_GPS;
  }
}

static void sendSampleFrame(bool test) {
  fbStart(SCI_T_SAMPLE);
  fbU32(S.seq); fbU32(S.t_ms); fbU32(test ? 1 : 0); fbHex(S.valid); fbHex(S.oor); fbHex(S.synth);
  uint16_t have = S.valid | S.synth;
  auto has = [&](uint8_t sb) { return (have >> sb) & 1; };
  fbF(S.t_mcp[0], 2, has(SB_MCP1)); fbF(S.t_mcp[1], 2, has(SB_MCP2)); fbF(S.t_mcp[2], 2, has(SB_MCP3));
  fbF(S.t_tmp119, 3, has(SB_TMP119));
  if (has(SB_VEML)) { fbU32(S.veml_als); fbU32(S.veml_white); fbF(S.veml_lux, 1, true); } else { fbSep(); fbSep(); fbSep(); }
  if (has(SB_LTR)) fbU32(S.ltr_uvs); else fbSep();
  fbF(S.ms_p, 2, has(SB_MS_PT)); fbF(S.ms_t, 2, has(SB_MS_PT)); fbF(S.ms_rh, 1, has(SB_MS_RH));
  fbF(S.aht_t, 2, has(SB_AHT)); fbF(S.aht_rh, 1, has(SB_AHT));
  fbF(S.ax, 3, has(SB_LIS)); fbF(S.ay, 3, has(SB_LIS)); fbF(S.az, 3, has(SB_LIS));
  fbF(S.bmp_p, 2, has(SB_BMP)); fbF(S.bmp_t, 2, has(SB_BMP));
  bool g = has(SB_GPS);
  fbU32(S.gps_fix);
  if (g) { fbU32(S.gps_qual); fbU32(S.gps_sats); } else { fbSep(); fbSep(); }
  bool fix = g && S.gps_fix;
  fbF(S.gps_lat, 6, fix); fbF(S.gps_lon, 6, fix); fbF(S.gps_alt, 1, fix);
  if (fix) { char t[8]; snprintf(t, sizeof t, "%06lu", (unsigned long)S.gps_utc); fbRaw(t); fbSep(); fbU32(S.gps_date); }
  else { fbSep(); fbSep(); }
  fbF(S.gps_hdop, 2, fix);
  if (g) fbU32(S.gps_age, false);
  linkSend(fbSeal());
}

static void printStatus() {
  Serial.print(F("[status] up=")); Serial.print(millis() / 1000); Serial.print(F("s seq=")); Serial.print(g_seq);
  Serial.print(F(" test=")); Serial.print(testModeActive());
  Serial.print(F(" det=0x")); Serial.print(det_mask, HEX);
  Serial.print(F(" valid=0x")); Serial.print(S.valid, HEX);
  Serial.print(F(" oor=0x")); Serial.print(S.oor, HEX);
  Serial.print(F(" i2c_to=")); Serial.print(i2c_timeouts);
  Serial.print(F(" mux_fail=")); Serial.print(mux_fail); Serial.print(F(" read_fail=")); Serial.print(read_fail); Serial.print(F(" trunc=")); Serial.print(frames_trunc);
  Serial.print(F(" gps_fix=")); Serial.print(S.gps_fix); Serial.print(F(" sats=")); Serial.print(S.gps_sats);
  Serial.print(F(" nmea=")); Serial.print(gps_sentences);
  Serial.print(F(" | i2c tx=")); Serial.print(frames_sent);
  Serial.print(F(" drop=")); Serial.print(frames_drop);
  Serial.print(F(" chunks=")); Serial.print(chunks_sent);
  Serial.print(F(" err=")); Serial.print(i2c_link_fail);
  Serial.print(F(" ack=")); Serial.print(ack_count);
  Serial.print(F(" uno_sd=")); Serial.print(uno_sd_ok);
  if (millis() - ack_last_ms > 10000UL) Serial.print(F(" **NO UNO ACK**"));
  Serial.println();
}

static void handleUsbCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') { test_mode_cmd = !test_mode_cmd; Serial.print(F("test_mode_cmd=")); Serial.println(test_mode_cmd); }
    else if (c == 'd') { detectAll(false); printDetectTable(); sendDetectFrame(); }
    else if (c == 's') printStatus();
    else if (c == 'w') { Serial.println(F("WATCHDOG TEST: hanging now; board must reboot within 4 s")); Serial.flush(); for (;;) {} }
    else if (c == 'h') Serial.println(F("cmds: t=toggle test mode  d=re-detect  s=status  w=watchdog hang test"));
  }
}

void setup() {
  reset_cause = MCUSR; MCUSR = 0; wdt_disable();
  pinMode(TEST_MODE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(USB_BAUD);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
  delay(5500);

  Serial.println();
  Serial.print(F("=== ")); Serial.print(F(FW_NAME)); Serial.print(F(" ")); Serial.print(F(FW_VERSION));
  Serial.print(F("  link v")); Serial.print(SCI_LINK_VERSION); Serial.print(F("  reset_cause=0x")); Serial.println(reset_cause, HEX);

  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(50);
  detectAll(false);
  printDetectTable();
  Serial.print(F("test mode: ")); Serial.println(testModeActive() ? F("ON (jumper)") : F("off"));

  sendBootFrame();
  sendDetectFrame();
  next_sample_ms = millis() + 500;
  next_status_ms = millis() + STATUS_PERIOD_MS;
  next_reprobe_ms = millis() + REPROBE_PERIOD_MS;
#if USE_WATCHDOG
  wdt_enable(WDTO_4S);
#endif
}

void loop() {
  wdt_reset();
  gpsPump();
  linkPump();
  handleUsbCommand();
  uint32_t now = millis();

  if (!sweep_active && (int32_t)(now - next_sample_ms) >= 0) {
    sweep_active = true; sweep_step = 0; sweep_t0 = now;
    next_sample_ms += SAMPLE_PERIOD_MS;
    if ((int32_t)(now - next_sample_ms) > 0) next_sample_ms = now + SAMPLE_PERIOD_MS;  // overran -> resync, no catch-up burst
  }
  if (sweep_active) {
    if (sweepStep()) {
      sweep_active = false;
      bool test = testModeActive();
      if (test) applyTestData();
      sendSampleFrame(test);
      g_seq++;
      digitalWrite(LED_BUILTIN, g_seq & 1);
    }
  }
  if ((int32_t)(now - next_status_ms) >= 0) { next_status_ms += STATUS_PERIOD_MS; printStatus(); }
  if ((int32_t)(now - next_reprobe_ms) >= 0 && !sweep_active) {
    next_reprobe_ms += REPROBE_PERIOD_MS;
    uint16_t before = det_mask;
    detectAll(true);
    if (det_mask != before) { Serial.println(F("[reprobe] sensor set changed")); printDetectTable(); sendDetectFrame(); }
  }
}
