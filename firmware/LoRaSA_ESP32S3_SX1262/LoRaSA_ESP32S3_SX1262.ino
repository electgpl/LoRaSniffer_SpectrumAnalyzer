/**
 * LoRa Spectrum Analyzer - ESP32-S3 + SX1262 (XTAL variant)
 *
 * Swept-tuned spectrum analyzer built on the SX1262 instantaneous RSSI.
 *
 * Measurement principle
 *   - The modem runs in GFSK mode: the GFSK channel filter (RxBw, 4.8-467 kHz)
 *     is used as the resolution bandwidth (RBW).
 *   - Per point: SetStandby(XOSC) -> SetRfFrequency -> SetRx(continuous)
 *     -> settling delay -> N x GetRssiInst (0x15, P[dBm] = -RssiInst/2).
 *   - Two detectors per point over the N samples:
 *       pk = maximum (peak detector, catches short bursts / chirps)
 *       av = mean in linear power (averaging in dB would bias noise low by ~2.5 dB)
 *
 * The sweep path uses raw SPI commands for speed and to stay independent of
 * RadioLib version changes (RSSI fix in 7.0.2, automatic image calibration on
 * frequency change since 7.1.0). RadioLib is only used for initialization.
 *
 * Serial protocol (one JSON object per line)
 *   Out:
 *     {"device":"electgpl-lora-sa","meta":true,...}            boot / after CFG
 *     {"device":"electgpl-lora-sa","log":"INFO","text":"..."}  events
 *     {"device":"electgpl-lora-sa","sw":N,"f0":MHz,"st":kHz,"n":pts,
 *      "t_us":duration,"pk":"HEX","av":"HEX"}                   one sweep
 *       pk/av: one byte per point, value = -2 * P[dBm] (native RssiInst format)
 *   In (text, newline terminated):
 *     CFG <f0_MHz> <f1_MHz> <step_kHz> <rbw_kHz> <N>
 *     SETTLE <us>    (0 = automatic)
 *     PPM <ppm>      (32 MHz crystal correction, signed)
 *     RUN | STOP | META
 *
 * Board: ESP32S3 Dev Module, 240 MHz
 *   - Native USB (GPIO19/20): "USB CDC On Boot: Enabled" (baud rate ignored).
 *   - USB-UART bridge on UART0: "USB CDC On Boot: Disabled", SERIAL_BAUD 921600.
 *
 * Author: Electgpl
 * License: MIT
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>

// ============================================================
// CONFIGURATION
// ============================================================
#define FW_VERSION          "1.0"
#define DEVICE_STR          "electgpl-lora-sa"
#define SERIAL_BAUD         921600   // ignored on native USB CDC

// Default sweep (overridden by the dashboard on connect)
#define DEF_F0_MHZ          902.0
#define DEF_F1_MHZ          928.0
#define DEF_STEP_KHZ        50.0
#define DEF_RBW_KHZ         117.3
#define DEF_NSAMP           4

#define MAX_PTS             2048
#define MAX_NSAMP           64
#define F_MIN_MHZ           150.0     // SX1262 synthesizer range
#define F_MAX_MHZ           960.0

// SX1262 regulator: 0 = DC-DC, 1 = LDO (removes DC-DC switching spurs, ~2x RX current)
#define USE_LDO             0
// RX boosted gain (reg 0x08AC = 0x96): better NF, worse strong-signal handling
#define RX_BOOSTED_GAIN     1

// Spans wider than this get image calibration per segment during the sweep
#define CAL_SEG_MHZ         32.0

// Automatic settling heuristic (validate on the bench with a CW source):
//   settle = SETTLE_BASE_US + SETTLE_K / RBW[Hz]   [us]
//   gap    = 1e6 / RBW[Hz] between RSSI reads
#define SETTLE_BASE_US      150
#define SETTLE_K            8.0e6

// SX126x supports up to 16 MHz SPI. Its harmonics fall in-band (n x f_SPI):
// at 8 MHz expect spurs at 904 / 912 / 920 / 928 MHz.
#define SPI_RAW_HZ          8000000

// ============================================================
// PINOUT
// ============================================================
#define PIN_SCK     12
#define PIN_MISO    13
#define PIN_MOSI    11
#define PIN_CS      10
#define PIN_RST     18
#define PIN_BUSY     4
#define PIN_DIO1    14
#define PIN_RFSW     5      // RF switch, LOW = RX (DIO2 not used)
#define PIN_LED_RED 47
#define LED_ON      HIGH
#define LED_OFF     LOW

// ============================================================
// SX126x opcodes used by the raw path
// ============================================================
#define OP_SET_STANDBY      0x80
#define OP_SET_RX           0x82
#define OP_SET_RF_FREQ      0x86
#define OP_CALIBRATE_IMAGE  0x98
#define OP_GET_RSSI_INST    0x15
#define STDBY_RC            0x00
#define STDBY_XOSC          0x01

static const float RBW_TABLE_KHZ[] = {
  4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9,
  58.6, 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6, 467.0
};
static const int RBW_N = sizeof(RBW_TABLE_KHZ) / sizeof(RBW_TABLE_KHZ[0]);

// ============================================================
// HARDWARE / STATE
// ============================================================
SPIClass spi_lora(FSPI);
SX1262   radio = new Module(PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY, spi_lora);
static const SPISettings spi_raw(SPI_RAW_HZ, MSBFIRST, SPI_MODE0);

struct SweepCfg {
  double   f0_mhz, step_khz;
  uint16_t npts;
  float    rbw_khz;
  uint8_t  nsamp;
  uint32_t settle_us;
  uint32_t gap_us;
  bool     settle_auto;
  float    ppm;
  uint8_t  nseg;           // image calibration segments
} cfg;

static bool     running   = true;
static uint32_t sweep_seq = 0;
static int16_t  cur_seg   = -1;

static uint8_t  buf_pk[MAX_PTS];
static uint8_t  buf_av[MAX_PTS];
static float    lut_lin[256];            // lut_lin[r] = 10^(-r/20), relative power
static char     line_out[4 * MAX_PTS + 256];
static char     line_in[160];
static uint8_t  line_in_len = 0;
static bool     cmd_pending = false;

static const char HEXC[] = "0123456789ABCDEF";

// ============================================================
// RAW SPI
// ============================================================
static inline bool wait_busy(uint32_t timeout_us = 30000) {
  uint32_t t0 = micros();
  while (digitalRead(PIN_BUSY)) {
    if ((uint32_t)(micros() - t0) > timeout_us) return false;
  }
  return true;
}

// BUSY rises a few hundred ns after NSS goes high; the 1 us delay prevents
// sampling a stale BUSY level on the next command.
static bool sx_cmd(uint8_t op, const uint8_t* p, uint8_t n) {
  if (!wait_busy()) return false;
  spi_lora.beginTransaction(spi_raw);
  digitalWrite(PIN_CS, LOW);
  spi_lora.transfer(op);
  for (uint8_t i = 0; i < n; i++) spi_lora.transfer(p[i]);
  digitalWrite(PIN_CS, HIGH);
  spi_lora.endTransaction();
  delayMicroseconds(1);
  return true;
}

static int sx_rssi_raw() {
  if (!wait_busy()) return -1;
  spi_lora.beginTransaction(spi_raw);
  digitalWrite(PIN_CS, LOW);
  spi_lora.transfer(OP_GET_RSSI_INST);
  spi_lora.transfer(0x00);                 // status
  uint8_t r = spi_lora.transfer(0x00);     // RssiInst
  digitalWrite(PIN_CS, HIGH);
  spi_lora.endTransaction();
  delayMicroseconds(1);
  return r;
}

static inline void sx_standby(uint8_t mode) { sx_cmd(OP_SET_STANDBY, &mode, 1); }

static void sx_set_freq(double f_mhz) {
  // If the crystal runs +p ppm high, the actual RF is f_set * (1 + p),
  // so f / (1 + p) is programmed. Synthesizer step: 32 MHz / 2^25 = 0.954 Hz.
  double f_hz = f_mhz * 1e6 / (1.0 + (double)cfg.ppm * 1e-6);
  uint32_t frf = (uint32_t)llround(f_hz * 33554432.0 / 32000000.0);
  uint8_t p[4] = { (uint8_t)(frf >> 24), (uint8_t)(frf >> 16),
                   (uint8_t)(frf >> 8),  (uint8_t)frf };
  sx_cmd(OP_SET_RF_FREQ, p, 4);
}

static void sx_rx_continuous() {
  const uint8_t p[3] = { 0xFF, 0xFF, 0xFF };
  sx_cmd(OP_SET_RX, p, 3);
}

static void sx_cal_image(double fmin_mhz, double fmax_mhz) {
  sx_standby(STDBY_RC);
  int a = (int)floor(fmin_mhz / 4.0);
  int b = (int)ceil(fmax_mhz / 4.0);
  if (a < 0) a = 0;
  if (b > 255) b = 255;
  if (b <= a) b = a + 1;
  uint8_t p[2] = { (uint8_t)a, (uint8_t)b };
  sx_cmd(OP_CALIBRATE_IMAGE, p, 2);
  wait_busy(50000);
  sx_standby(STDBY_XOSC);
}

// ============================================================
// HELPERS
// ============================================================
static float rbw_snap(float khz) {
  int best = 0; float d = 1e9f;
  for (int i = 0; i < RBW_N; i++) {
    float e = fabsf(RBW_TABLE_KHZ[i] - khz);
    if (e < d) { d = e; best = i; }
  }
  return RBW_TABLE_KHZ[best];
}

static double seg_lo(uint8_t s) {
  double span = (cfg.npts - 1) * cfg.step_khz / 1000.0;
  return cfg.f0_mhz + span * s / cfg.nseg;
}

static void log_msg(const char* cat, const char* text) {
  Serial.printf("{\"device\":\"" DEVICE_STR "\",\"log\":\"%s\",\"text\":\"%s\"}\n", cat, text);
}

static void print_meta() {
  double f1 = cfg.f0_mhz + (cfg.npts - 1) * cfg.step_khz / 1000.0;
  Serial.printf("{\"device\":\"" DEVICE_STR "\",\"meta\":true,\"fw\":\"" FW_VERSION "\","
                "\"f0\":%.6f,\"f1\":%.6f,\"st\":%.3f,\"n\":%u,\"rbw\":%.1f,\"ns\":%u,"
                "\"settle_us\":%lu,\"settle_auto\":%s,\"gap_us\":%lu,\"ppm\":%.2f,"
                "\"segs\":%u,\"reg\":\"%s\",\"boost\":%s,\"run\":%s}\n",
                cfg.f0_mhz, f1, cfg.step_khz, cfg.npts, cfg.rbw_khz, cfg.nsamp,
                (unsigned long)cfg.settle_us, cfg.settle_auto ? "true" : "false",
                (unsigned long)cfg.gap_us, cfg.ppm, cfg.nseg,
                USE_LDO ? "LDO" : "DC-DC", RX_BOOSTED_GAIN ? "true" : "false",
                running ? "true" : "false");
}

static void recompute_timing(uint32_t settle_req_us) {
  double rbw_hz = cfg.rbw_khz * 1000.0;
  cfg.settle_auto = (settle_req_us == 0);
  cfg.settle_us = cfg.settle_auto ? (uint32_t)(SETTLE_BASE_US + SETTLE_K / rbw_hz)
                                  : settle_req_us;
  double gap = 1e6 / rbw_hz - 15.0;        // ~15 us already spent in the SPI transaction
  cfg.gap_us = gap > 0 ? (uint32_t)gap : 0;
}

static bool apply_cfg(double f0, double f1, double step_khz, float rbw_khz, int ns) {
  if (f0 < F_MIN_MHZ || f1 > F_MAX_MHZ || f1 <= f0) return false;
  if (step_khz < 1.0 || ns < 1 || ns > MAX_NSAMP) return false;
  long npts = (long)floor((f1 - f0) * 1000.0 / step_khz + 1e-6) + 1;
  if (npts < 2 || npts > MAX_PTS) return false;

  float rbw = rbw_snap(rbw_khz);
  radio.standby();
  int s = radio.setRxBandwidth(rbw);
  if (s != RADIOLIB_ERR_NONE) {
    char t[64]; snprintf(t, sizeof(t), "setRxBandwidth err=%d", s);
    log_msg("ERR", t);
    return false;
  }
  cfg.f0_mhz = f0; cfg.step_khz = step_khz; cfg.npts = (uint16_t)npts;
  cfg.rbw_khz = rbw; cfg.nsamp = (uint8_t)ns;
  recompute_timing(cfg.settle_auto ? 0 : cfg.settle_us);

  double span = (npts - 1) * step_khz / 1000.0;
  cfg.nseg = (uint8_t)ceil(span / CAL_SEG_MHZ);
  if (cfg.nseg < 1) cfg.nseg = 1;
  cur_seg = -1;
  if (cfg.nseg == 1) { sx_cal_image(f0, f0 + span); cur_seg = 0; }
  return true;
}

// ============================================================
// COMMANDS
// ============================================================
static void poll_serial() {
  while (Serial.available() && !cmd_pending) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { line_in[line_in_len] = 0; if (line_in_len) cmd_pending = true; break; }
    if (line_in_len < sizeof(line_in) - 1) line_in[line_in_len++] = c;
  }
}

static void handle_cmd() {
  cmd_pending = false;
  char* l = line_in;
  line_in_len = 0;
  double a, b, c; float d, e; int n;

  if (sscanf(l, "CFG %lf %lf %lf %f %d", &a, &b, &c, &d, &n) == 5) {
    if (apply_cfg(a, b, c, d, n)) print_meta();
    else log_msg("ERR", "invalid CFG (150-960 MHz, step>=1 kHz, pts<=2048, N 1-64)");
  } else if (sscanf(l, "SETTLE %f", &d) == 1) {
    recompute_timing(d <= 0 ? 0 : (uint32_t)d);
    print_meta();
  } else if (sscanf(l, "PPM %f", &e) == 1) {
    cfg.ppm = e;
    print_meta();
  } else if (!strcmp(l, "RUN"))  { running = true;  print_meta(); }
  else if (!strcmp(l, "STOP"))   { running = false; sx_standby(STDBY_RC); print_meta(); }
  else if (!strcmp(l, "META"))   { print_meta(); }
  else log_msg("ERR", "unknown command");
}

// ============================================================
// SWEEP
// ============================================================
// Returns false if aborted by an incoming command.
static bool do_sweep(uint32_t* dur_us) {
  uint32_t t0 = micros();
  for (uint16_t i = 0; i < cfg.npts; i++) {
    double f = cfg.f0_mhz + i * cfg.step_khz / 1000.0;

    if (cfg.nseg > 1) {
      int16_t s = (int16_t)((i * (uint32_t)cfg.nseg) / cfg.npts);
      if (s != cur_seg) {
        sx_cal_image(seg_lo(s), seg_lo(s + 1));
        cur_seg = s;
      }
    }

    sx_standby(STDBY_XOSC);
    sx_set_freq(f);
    sx_rx_continuous();
    delayMicroseconds(cfg.settle_us);

    int   rmin = 255;
    float acc  = 0.0f;
    for (uint8_t k = 0; k < cfg.nsamp; k++) {
      int r = sx_rssi_raw();
      if (r < 0) r = 255;                     // BUSY timeout -> floor
      if (r < rmin) rmin = r;                 // lower raw = higher power
      acc += lut_lin[r];
      if (cfg.gap_us && k + 1 < cfg.nsamp) delayMicroseconds(cfg.gap_us);
    }
    float avg_dbm = 10.0f * log10f(acc / cfg.nsamp);
    int   ra = (int)lroundf(-2.0f * avg_dbm);
    buf_pk[i] = (uint8_t)rmin;
    buf_av[i] = (uint8_t)constrain(ra, 0, 255);

    if ((i & 31) == 31) {                    // stay responsive on long sweeps
      poll_serial();
      if (cmd_pending) return false;
    }
  }
  *dur_us = micros() - t0;
  return true;
}

static void emit_sweep(uint32_t dur_us) {
  int p = snprintf(line_out, sizeof(line_out),
                   "{\"device\":\"" DEVICE_STR "\",\"sw\":%lu,\"f0\":%.6f,\"st\":%.3f,"
                   "\"n\":%u,\"t_us\":%lu,\"pk\":\"",
                   (unsigned long)sweep_seq, cfg.f0_mhz, cfg.step_khz, cfg.npts,
                   (unsigned long)dur_us);
  for (uint16_t i = 0; i < cfg.npts; i++) {
    line_out[p++] = HEXC[buf_pk[i] >> 4]; line_out[p++] = HEXC[buf_pk[i] & 0xF];
  }
  p += snprintf(line_out + p, sizeof(line_out) - p, "\",\"av\":\"");
  for (uint16_t i = 0; i < cfg.npts; i++) {
    line_out[p++] = HEXC[buf_av[i] >> 4]; line_out[p++] = HEXC[buf_av[i] & 0xF];
  }
  p += snprintf(line_out + p, sizeof(line_out) - p, "\"}\n");
  Serial.write((const uint8_t*)line_out, p);
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin(SERIAL_BAUD);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) {}

  pinMode(PIN_LED_RED, OUTPUT); digitalWrite(PIN_LED_RED, LED_OFF);
  pinMode(PIN_RFSW, OUTPUT);    digitalWrite(PIN_RFSW, LOW);   // RX only

  for (int r = 0; r < 256; r++) lut_lin[r] = powf(10.0f, -r / 20.0f);

  spi_lora.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  // GFSK: bit rate and deviation are irrelevant for power measurement.
  // tcxoVoltage = 0 -> XTAL module (the 1.6 V default returns -707).
  int s = radio.beginFSK(DEF_F0_MHZ, 4.8, 5.0, DEF_RBW_KHZ, 0, 16, 0.0f, USE_LDO ? true : false);
  if (s != RADIOLIB_ERR_NONE) {
    while (true) {
      Serial.printf("{\"device\":\"" DEVICE_STR "\",\"log\":\"FATAL\",\"text\":\"beginFSK=%d\"}\n", s);
      digitalWrite(PIN_LED_RED, LED_ON); delay(100);
      digitalWrite(PIN_LED_RED, LED_OFF); delay(900);
    }
  }
  radio.setDio2AsRfSwitch(false);
#if RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(true);
#endif

  cfg.ppm = 0.0f;
  cfg.settle_auto = true;
  apply_cfg(DEF_F0_MHZ, DEF_F1_MHZ, DEF_STEP_KHZ, DEF_RBW_KHZ, DEF_NSAMP);
  log_msg("INFO", "LoRa SA ready - SX1262 XTAL, GFSK modem, instantaneous RSSI");
  print_meta();
}

void loop() {
  poll_serial();
  if (cmd_pending) handle_cmd();

  if (!running) { delay(5); return; }

  uint32_t dur;
  if (!do_sweep(&dur)) return;
  sweep_seq++;
  emit_sweep(dur);
  digitalWrite(PIN_LED_RED, (sweep_seq & 15) == 0 ? LED_ON : LED_OFF);
}
