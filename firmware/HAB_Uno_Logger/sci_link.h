// SCI-LINK v1: shared Mega/Uno frame format. identical copy in both sketch folders
// frame: $T,f0,f1,...*XXXX\n, XXXX = CRC-16/CCITT-FALSE over bytes between $ and *

#ifndef SCI_LINK_H
#define SCI_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SCI_LINK_VERSION      1
#define SCI_LINK_BAUD         38400UL  // not used by the I2C link
#define SCI_FRAME_MAX         250
#define SCI_LINE_BUF          256

#define SCI_T_SAMPLE   'S'
#define SCI_T_DETECT   'D'
#define SCI_T_BOOT     'B'
#define SCI_T_ACK      'A'

enum {
  SCI_F_SEQ = 0, SCI_F_MEGA_MS, SCI_F_TEST, SCI_F_VALID, SCI_F_OOR, SCI_F_SYNTH,
  SCI_F_T_MCP1, SCI_F_T_MCP2, SCI_F_T_MCP3, SCI_F_T_TMP119,
  SCI_F_VEML_ALS, SCI_F_VEML_WHITE, SCI_F_VEML_LUX, SCI_F_LTR_UVS,
  SCI_F_MS_P, SCI_F_MS_T, SCI_F_MS_RH, SCI_F_AHT_T, SCI_F_AHT_RH,
  SCI_F_LIS_AX, SCI_F_LIS_AY, SCI_F_LIS_AZ, SCI_F_BMP_P, SCI_F_BMP_T,
  SCI_F_GPS_FIX, SCI_F_GPS_QUAL, SCI_F_GPS_SATS, SCI_F_GPS_LAT, SCI_F_GPS_LON,
  SCI_F_GPS_ALT, SCI_F_GPS_UTC, SCI_F_GPS_DATE, SCI_F_GPS_HDOP, SCI_F_GPS_AGE,
  SCI_F_COUNT
};

#define SCI_S_BODY_HEADER \
  "seq,mega_ms,test,valid,oor,synth," \
  "t_mcp1_c,t_mcp2_c,t_mcp3_c,t_tmp119_c," \
  "veml_als_raw,veml_white_raw,veml_lux,ltr_uvs_raw," \
  "ms_p_hpa,ms_t_c,ms_rh_pct,aht_t_c,aht_rh_pct," \
  "lis_ax_g,lis_ay_g,lis_az_g,bmp_p_hpa,bmp_t_c," \
  "gps_fix,gps_qual,gps_sats,gps_lat,gps_lon,gps_alt_m,gps_utc,gps_date,gps_hdop,gps_age_ms"

enum {
  // bit positions for the valid / oor / synth masks
  SB_MCP1 = 0, SB_MCP2, SB_MCP3, SB_TMP119, SB_VEML, SB_LTR, SB_MS_PT, SB_MS_RH,
  SB_AHT, SB_LIS, SB_BMP, SB_GPS, SB_INA219, SB_MUX, SB_COUNT
};

static inline uint16_t sci_crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static inline int sci_hexval(char c) {  // uppercase only, so a flipped case bit fails validation
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static inline size_t sci_seal(char *line, size_t len, size_t cap) {
  if (len + 6 > cap || len < 2) return 0;
  uint16_t crc = sci_crc16((const uint8_t *)line + 1, len - 1);
  line[len++] = '*';
  for (int8_t sh = 12; sh >= 0; sh -= 4) {
    uint8_t n = (crc >> sh) & 0xF;
    line[len++] = (char)(n < 10 ? '0' + n : 'A' + n - 10);
  }
  line[len++] = '\n';
  line[len] = 0;
  return len;
}

enum {
  SCI_OK = 0,
  SCI_E_NOSTART = 1,
  SCI_E_SHORT   = 2,
  SCI_E_NOSTAR  = 3,
  SCI_E_HEX     = 4,
  SCI_E_CRC     = 5,
  SCI_E_TYPE    = 6,
  SCI_E_OVERFLOW= 7
};

static inline int sci_validate(char *line, size_t len, char *type,
                               char **body, size_t *body_len) {
  if (len < 8) return SCI_E_SHORT;
  if (line[0] != '$') return SCI_E_NOSTART;
  if (line[len - 5] != '*') return SCI_E_NOSTAR;
  int v = 0;
  for (uint8_t i = 0; i < 4; i++) {
    int h = sci_hexval(line[len - 4 + i]);
    if (h < 0) return SCI_E_HEX;
    v = (v << 4) | h;
  }
  uint16_t crc = sci_crc16((const uint8_t *)line + 1, len - 6);
  if ((uint16_t)v != crc) return SCI_E_CRC;
  if (line[1] < 'A' || line[1] > 'Z' || line[2] != ',') return SCI_E_TYPE;
  line[len - 5] = 0;
  *type = line[1];
  *body = line + 3;
  *body_len = len - 8;
  return SCI_OK;
}

static inline int sci_field(const char *body, uint8_t k, char *out, uint8_t outcap) {
  const char *p = body;
  while (k--) {
    p = strchr(p, ',');
    if (!p) return -1;
    p++;
  }
  uint8_t n = 0;
  while (*p && *p != ',' && n < outcap - 1) out[n++] = *p++;
  out[n] = 0;
  return n;
}

typedef struct {
  char     buf[SCI_LINE_BUF];
  uint16_t len;
  uint8_t  overflow;
  uint16_t overflow_count;
} sci_rx_t;

static inline void sci_rx_init(sci_rx_t *r) { r->len = 0; r->overflow = 0; r->overflow_count = 0; }

static inline int sci_rx_feed(sci_rx_t *r, char c) {
  if (c == '\n') {
    if (r->overflow) { r->overflow = 0; r->len = 0; return 0; }
    while (r->len && r->buf[r->len - 1] == '\r') r->len--;
    r->buf[r->len] = 0;
    if (r->len == 0) return 0;
    return 1;
  }
  if (r->overflow) return 0;
  if (c == '$') { r->len = 0; }  // '$' always restarts a frame: resync after garbage
  if (r->len >= SCI_LINE_BUF - 1) {
    r->overflow = 1; r->overflow_count++; r->len = 0; return -1;
  }
  r->buf[r->len++] = c;
  return 0;
}
static inline void sci_rx_reset(sci_rx_t *r) { r->len = 0; }

#endif
